import sys
import os
import time
import threading
import subprocess
import asyncio
import numpy as np
import requests
import edge_tts
import base64
import speech_recognition as sr
import google.generativeai as genai
import random
import cv2 
import re
import json
import math
from PIL import Image
from flask import Flask, render_template_string, request, jsonify, send_file, Response

# ALSA hatalarını bastır
sys.stderr = open(os.devnull, 'w')

# --- MODÜLER İMPORTLAR ---
try:
    from ServoController import ServoController
    import Locomotion  
    import RPi.GPIO as GPIO
    from obstacle_avoidance import ObstacleAvoidance 
except ImportError:
    print("⚠️ Kütüphaneler eksik, simülasyon modu.")

# ==================================================================
# 1. AYARLAR
# ==================================================================
app = Flask(__name__)

# GEMINI
GEMINI_API_KEY = "Enter your Gemini API key"
GEMINI_MODEL = "models/gemini-robotics-er-1.5-preview"
genai.configure(api_key=GEMINI_API_KEY)

# YEREL LLM
LOCAL_LLM_URL = "Enter your LMstudio model link"  
LOCAL_LLM_MODEL = "qwen/qwen3-vl-30b"

locomotion = None
obstacle_avoider = None 
object_tracker = None 
rec_process = None
current_tts_process = None 

AUDIO_FILE = "mic_input.wav"
IMAGE_FILE = "ai_view.jpg"

global_jpeg_bytes = None 
last_frame_cv2 = None 

robot_state = {
    "trot": False, "balance": False, "mic": False, 
    "auto": False, "tracking": False, "approach": False,
    "target_name": "", "last_photo_time": 0,
    "log": ["Sistem: SMART HUNTER V6 Hazır."]
}

# ==================================================================
# 2. AKILLI HEDEF TAKİP SINIFI (SMART TRACKER)
# ==================================================================
class SmartColorTracker:
    def __init__(self, locomotion_obj):
        self.locomotion = locomotion_obj
        self.active = False
        self.bbox = None # (x, y, w, h)
        self.roi_hist = None
        
        # CamShift Ayarları
        self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
        
        # Ekran
        self.frame_w = 480
        self.frame_h = 360
        self.center_x = self.frame_w // 2
        
        # Filtreleme Değişkenleri
        self.last_center = (0, 0)
        
        # --- ADAPTİF MASKE AYARLARI ---
        self.lower_color = np.array((0., 60., 32.)) 
        self.upper_color = np.array((180., 255., 255.))
        self.target_type = "Renkli" 
        
        # Scanner (Arama) Ayarları
        self.scan_x = 0; self.scan_y = 0
        self.scan_speed = 20; self.scan_step = 60
        self.scan_w = 100; self.scan_h = 100
        self.searching = False
        self.thread = None

    def auto_tune_mask(self, roi_bgr):
        """Rengi analiz et ve maskeyi ayarla"""
        hsv_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        mean_saturation = np.mean(hsv_roi[:, :, 1])
        mean_value = np.mean(hsv_roi[:, :, 2])
        
        print(f"🎨 Renk Analizi -> Sat: {mean_saturation:.1f}, Val: {mean_value:.1f}")

        if mean_value < 50: 
            self.target_type = "Karanlık/Siyah"
            self.lower_color = np.array((0., 0., 0.)) 
            self.upper_color = np.array((180., 255., 80.)) 

        elif mean_saturation < 40 and mean_value > 100:
            self.target_type = "Beyaz/Gri"
            self.lower_color = np.array((0., 0., 150.)) 
            self.upper_color = np.array((180., 60., 255.))

        else:
            self.target_type = "Canlı Renk"
            self.lower_color = np.array((0., 60., 32.))
            self.upper_color = np.array((180., 255., 255.))
            
        print(f"✅ Algılanan Tip: {self.target_type}")

    def get_clean_mask(self, hsv_frame):
        mask = cv2.inRange(hsv_frame, self.lower_color, self.upper_color)
        return mask

    def init_tracking(self, frame, target_name):
        print(f"🎯 Hedef Aranıyor: {target_name}")
        self.frame_h, self.frame_w, _ = frame.shape
        
        try:
            pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            model = genai.GenerativeModel(GEMINI_MODEL)
            prompt = f"Detect '{target_name}'. Return JSON list: [{{'ymin': int, 'xmin': int, 'ymax': int, 'xmax': int}}] Coordinates 0-1000."
            
            response = model.generate_content([prompt, pil_img])
            text = response.text.strip().replace('```json', '').replace('```', '').strip()
            
            ymin, xmin, ymax, xmax = 0, 0, 0, 0
            found = False
            
            try:
                data = json.loads(text)
                if isinstance(data, list) and len(data) > 0:
                    obj = data[0]
                    ymin, xmin, ymax, xmax = obj['ymin'], obj['xmin'], obj['ymax'], obj['xmax']
                    found = True
            except:
                match = re.search(r'\[(.*?)\]', text)
                if match:
                    coords = [int(x.strip()) for x in match.group(1).split(',')]
                    if len(coords) >= 4:
                        ymin, xmin, ymax, xmax = coords[:4]
                        found = True

            if found:
                x = int(xmin / 1000 * self.frame_w)
                y = int(ymin / 1000 * self.frame_h)
                w = int((xmax - xmin) / 1000 * self.frame_w)
                h = int((ymax - ymin) / 1000 * self.frame_h)
                
                self.bbox = (x, y, w, h)
                self.last_center = (x + w//2, y + h//2)
                
                # --- AKILLI MASKELEME ---
                roi = frame[y:y+h, x:x+w]
                self.auto_tune_mask(roi)
                
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask = self.get_clean_mask(hsv_roi)
                
                self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
                cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
                
                self.active = True
                self.searching = False
                robot_state["approach"] = False
                self.scan_x = x; self.scan_y = y
                
                if not self.locomotion._standing:
                    self.locomotion.toggle_standing()
                    robot_state["trot"] = True
                
                if self.thread is None or not self.thread.is_alive():
                    self.thread = threading.Thread(target=self._tracking_loop, daemon=True)
                    self.thread.start()
                return True
            return False
        except Exception as e:
            print(f"Hata: {e}")
            return False

    def stop(self):
        self.active = False
        self.searching = False
        robot_state["approach"] = False
        self.bbox = None
        self.locomotion.set_rotation_factor(0.0)
        self.locomotion.set_forward_factor(0.0)

    def update_scan_box(self):
        self.scan_x += self.scan_speed
        if self.scan_x + self.scan_w >= self.frame_w:
            self.scan_x = 0; self.scan_y += self.scan_step
            if self.scan_y + self.scan_h >= self.frame_h:
                self.scan_y = 0

    def update(self, frame):
        if not self.active or self.roi_hist is None: return frame
        
        if not self.searching:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0, 180], 1)
            mask = self.get_clean_mask(hsv)
            dst &= mask 
            
            ret, new_bbox = cv2.CamShift(dst, self.bbox, self.term_crit)
            pts = cv2.boxPoints(ret)
            pts = np.intp(pts)
            x, y, w, h = new_bbox
            cx, cy = int(ret[0][0]), int(ret[0][1])
            area = w * h
            
            dist = math.hypot(cx - self.last_center[0], cy - self.last_center[1])
            is_jump = dist > 150 
            is_noise = area < 200 or area > (self.frame_w * self.frame_h * 0.9)
            
            if not is_jump and not is_noise:
                self.bbox = new_bbox
                self.last_center = (cx, cy)
                
                color = (0, 255, 0) if self.target_type == "Canlı Renk" else (200, 200, 200)
                img2 = cv2.polylines(frame, [pts], True, color, 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(frame, (self.center_x, self.frame_h//2), (cx, cy), (0, 255, 255), 2)
                
                info_text = f"{robot_state['target_name']} ({self.target_type})"
                cv2.putText(frame, info_text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(frame, f"X: {cx - self.center_x}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                print(f"⚠️ Kayıp! (Jump: {is_jump}, Noise: {is_noise})")
                self.searching = True
                self.locomotion.set_forward_factor(0.0)
                self.locomotion.set_rotation_factor(0.0)

        else:
            self.update_scan_box()
            sx, sy, sw, sh = int(self.scan_x), int(self.scan_y), self.scan_w, self.scan_h
            cv2.rectangle(frame, (sx, sy), (sx + sw, sy + sh), (255, 255, 0), 2)
            cv2.putText(frame, "TARANIYOR...", (sx, sy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            hsv_scan = cv2.cvtColor(frame[sy:sy+sh, sx:sx+sw], cv2.COLOR_BGR2HSV)
            if hsv_scan.size > 0:
                scan_mask = self.get_clean_mask(hsv_scan)
                scan_hist = cv2.calcHist([hsv_scan], [0], scan_mask, [180], [0, 180])
                cv2.normalize(scan_hist, scan_hist, 0, 255, cv2.NORM_MINMAX)
                
                score = cv2.compareHist(self.roi_hist, scan_hist, cv2.HISTCMP_CORREL)
                cv2.putText(frame, f"Match: {score:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                threshold = 0.55 if self.target_type != "Canlı Renk" else 0.65
                
                if score > threshold:
                    print("HEDEF YENİDEN BULUNDU!")
                    self.bbox = (sx, sy, sw, sh)
                    self.last_center = (sx + sw//2, sy + sh//2)
                    self.searching = False

        return frame

    def _tracking_loop(self):
        print("Takip Motoru Aktif")
        THRESHOLD_X = 60
        
        while self.active:
            if not self.searching and self.bbox:
                x, y, w, h = self.bbox
                cx = x + w // 2
                error_x = cx - self.center_x
                
                if abs(error_x) > THRESHOLD_X:
                    rot = 0.5 if error_x > 0 else -0.5
                    self.locomotion.set_rotation_factor(rot)
                    self.locomotion.set_forward_factor(0.0)
                else:
                    self.locomotion.set_rotation_factor(0.0)
                    if robot_state["approach"]:
                        if w * h > (self.frame_w * self.frame_h) * 0.45:
                             self.locomotion.set_forward_factor(0.0)
                        else:
                             self.locomotion.set_forward_factor(0.5)
                    else:
                        self.locomotion.set_forward_factor(0.0)
            else:
                self.locomotion.set_rotation_factor(0.0)
                self.locomotion.set_forward_factor(0.0)
            time.sleep(0.05)

# ==================================================================
# 3. ROBOT BAŞLATMA
# ==================================================================
print("Robot başlatılıyor...")
try:
    front_controller = ServoController(address=0x40) 
    back_controller = ServoController(address=0x41)  
    try: offsets = np.loadtxt("config.csv", delimiter=",", dtype=int)
    except: offsets = [0]*12
    fr_servos = front_controller.load_servos([0, 1, 2], offsets[0:3])
    fl_servos = front_controller.load_servos([13, 14, 15], offsets[3:6])
    br_servos = back_controller.load_servos([0, 1, 2], offsets[6:9])
    bl_servos = back_controller.load_servos([13, 14, 15], offsets[9:12])
    servos = fr_servos + fl_servos + br_servos + bl_servos
    locomotion = Locomotion.Locomotion(servos)
    for s in servos:
        s.set_min_angle(22.5); s.set_min_pulse(0.5 / ((1/50.0)/4096 * 1e3))
        s.set_max_angle(157.5); s.set_max_pulse(2.5 / ((1/50.0)/4096 * 1e3))
    
    obstacle_avoider = ObstacleAvoidance(locomotion)
    object_tracker = SmartColorTracker(locomotion)
    print("Robot hazır.")
except:
    class DummyLocomotion:
        async def Run(self): 
            while True: await asyncio.sleep(1)
        def set_forward_factor(self, x): pass
        def set_rotation_factor(self, x): pass
        def set_strafe_factor(self, x): pass
        def set_height_offset(self, x): pass
        def set_lean(self, x): pass 
        def set_balance_mode(self, x): return True
        def toggle_standing(self): pass
        def Shutdown(self): pass
        _standing = False
        _height_factor = 20.0
    locomotion = DummyLocomotion()
    obstacle_avoider = ObstacleAvoidance(locomotion)
    object_tracker = SmartColorTracker(locomotion)

def start_robot_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(locomotion.Run())
threading.Thread(target=start_robot_loop, daemon=True).start()

# ==================================================================
# 4. YARDIMCI FONKSİYONLAR
# ==================================================================
def add_log(sender, msg):
    entry = f"{sender}: {msg}"
    print(entry)
    robot_state["log"].append(entry)
    if len(robot_state["log"]) > 10: robot_state["log"].pop(0)

def stop_audio_playback():
    global current_tts_process
    if current_tts_process:
        try: current_tts_process.terminate(); current_tts_process = None
        except: pass

def generate_frames():
    global global_jpeg_bytes, last_frame_cv2
    cmd = ["rpicam-vid", "-t", "0", "--inline", "-n", "--width", "480", "--height", "360", "--framerate", "30", "--codec", "mjpeg", "-o", "-"]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**8)
    buffer = b""
    try:
        while True:
            chunk = process.stdout.read(4096)
            if not chunk: break
            buffer += chunk
            a = buffer.find(b'\xff\xd8')
            b = buffer.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = buffer[a:b+2]; buffer = buffer[b+2:]
                
                nparr = np.frombuffer(jpg, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                last_frame_cv2 = frame.copy()
                
                if robot_state["tracking"] and object_tracker.active:
                    frame = object_tracker.update(frame)
                
                ret, jpeg = cv2.imencode('.jpg', frame)
                global_jpeg_bytes = jpeg.tobytes()
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + global_jpeg_bytes + b'\r\n')
    finally: process.terminate()

def save_current_frame_for_ai():
    global last_frame_cv2
    if last_frame_cv2 is not None:
        cv2.imwrite(IMAGE_FILE, last_frame_cv2)
        robot_state["last_photo_time"] = time.time()
        return True
    return False

def image_to_base64():
    try:
        with open(IMAGE_FILE, "rb") as f: return base64.b64encode(f.read()).decode('utf-8')
    except: return ""

def send_local_llm_request(text, use_image=False):
    payload = { "model": LOCAL_LLM_MODEL, "temperature": 0.7, "stream": False, "messages": [{"role": "user", "content": []}] }
    content = payload["messages"][0]["content"]
    if use_image:
        img = image_to_base64()
        if img: content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img}"}})
    content.append({"type": "text", "text": text})
    try:
        res = requests.post(LOCAL_LLM_URL, json=payload, headers={"Content-Type": "application/json"}, timeout=60)
        if res.status_code == 200: return res.json()["choices"][0]["message"]["content"]
    except Exception as e: return f"Hata: {e}"
    return "Sunucu Hatası."

def speak_sync_interruptible(text):
    global current_tts_process
    try:
        edge_tts.Communicate(text, "tr-TR-AhmetNeural").save_sync("response.mp3")
        current_tts_process = subprocess.Popen(["mpg321", "response.mp3", "-q"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        current_tts_process.wait()
    except: pass
    finally: current_tts_process = None

def process_ai_interaction(text, source="Ses"):
    add_log(f"Siz ({source})", text)
    use_cam = "gör" in text.lower() or "bak" in text.lower()
    if use_cam: 
        add_log("Sistem", "📸 Görüntü alınıyor...")
        if save_current_frame_for_ai(): add_log("Sistem", "Analiz ediliyor...")
    add_log("AI", "Düşünüyor...")
    response = send_local_llm_request(text, use_cam)
    add_log("AI", response)
    speak_sync_interruptible(response)

def process_audio_thread():
    r = sr.Recognizer()
    try:
        with sr.AudioFile(AUDIO_FILE) as source:
            text = r.recognize_google(r.record(source), language="tr-TR")
            process_ai_interaction(text, "Ses")
    except Exception as e: add_log("Sistem", f"Hata: {e}")
    robot_state["mic"] = False 

def process_text_thread(text):
    process_ai_interaction(text, "Yazı")

# ==================================================================
# 5. WEB SAYFASI
# ==================================================================
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>SpotMicro Smart Hunter</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <style>
        body { background-color: #121212; color: white; font-family: sans-serif; text-align: center; margin: 0; user-select: none; }
        .video-container { width: 100%; max-width: 480px; margin: 0 auto; background: #000; border-bottom: 2px solid #333; position: relative; }
        .video-feed { width: 100%; height: auto; display: block; }
        .log-box { background: #1e1e1e; height: 60px; overflow-y: scroll; text-align: left; padding: 5px; font-size: 12px; color: #00ff00; font-family: monospace; border-bottom: 1px solid #333; }
        .chat-input-row { display: flex; justify-content: center; padding: 5px; background: #222; }
        .chat-input { flex: 1; padding: 10px; border-radius: 5px; border: 1px solid #555; background: #333; color: white; margin-right: 5px; }
        .chat-btn { width: 80px; border-radius: 5px; border: none; background: #2196F3; color: white; font-weight: bold; }
        .track-row { background: #263238; padding: 5px; display: flex; justify-content: center; gap: 10px; }
        .track-btn { background: #E91E63; color: white; border: none; padding: 10px; border-radius: 5px; font-weight: bold; flex: 1; }
        .approach-btn { background: #555; color: white; border: none; padding: 10px; border-radius: 5px; font-weight: bold; flex: 1; display: none; }
        .btn-row { display: flex; justify-content: center; margin: 5px 0; }
        .btn { width: 85px; height: 55px; margin: 4px; border: none; border-radius: 8px; font-weight: bold; font-size: 13px; color: white; box-shadow: 0 3px #000; transition: 0.1s; }
        .btn:active { transform: translateY(2px); box-shadow: 0 1px #000; }
        .c-blue { background-color: #2980b9; } .c-green { background-color: #27ae60; }
        .c-red { background-color: #c0392b; } .c-orange { background-color: #d35400; }
        .c-purple { background-color: #8e44ad; } .c-grey { background-color: #7f8c8d; } .c-teal { background-color: #009688; }
        .big-btn { width: 90px; height: 70px; font-size: 15px; }
    </style>
</head>
<body>
    <div class="video-container"><img src="/video_feed" class="video-feed"></div>
    <div class="track-row">
        <button id="btnTrack" class="track-btn" onclick="startTracking()">🎯 HEDEF TAKİP BAŞLAT</button>
        <button id="btnApproach" class="approach-btn" onclick="toggleApproach()">🚀 HEDEFE GİT</button>
    </div>
    <div class="log-box" id="logBox">Sistem Başlatılıyor...</div>
    <div class="chat-input-row">
        <input type="text" id="chatInput" class="chat-input" placeholder="Yerel AI'a yaz..." onkeypress="handleKeyPress(event)">
        <button class="chat-btn" onclick="sendText()">GÖNDER</button>
    </div>
    <div class="btn-row">
        <button id="btnMic" class="btn c-grey" onclick="toggleMic()">MİKROFON</button>
        <button class="btn c-red" onclick="sendCommand('cancel_mic')">DUR</button>
        <button id="btnBal" class="btn c-grey" onclick="toggleBalance()">DENGE</button>
    </div>
    <div class="btn-row">
        <button class="btn c-purple" ontouchstart="startMove('strafe_l')" ontouchend="stopMove()">SOL<br>KAY</button>
        <button class="btn c-blue" ontouchstart="startMove('forward')" ontouchend="stopMove()">İLERİ</button>
        <button class="btn c-purple" ontouchstart="startMove('strafe_r')" ontouchend="stopMove()">SAĞ<br>KAY</button>
    </div>
    <div class="btn-row">
        <button class="btn c-blue" ontouchstart="startMove('turn_l')" ontouchend="stopMove()">SOL<br>DÖN</button>
        <button id="btnTrot" class="btn c-red big-btn" onclick="toggleTrot()">TROT<br>BAŞLAT</button>
        <button class="btn c-blue" ontouchstart="startMove('turn_r')" ontouchend="stopMove()">SAĞ<br>DÖN</button>
    </div>
    <div class="btn-row">
        <button class="btn c-orange" ontouchstart="startMove('lean_l')" ontouchend="stopMove()">SOL<br>EĞİL</button>
        <button class="btn c-blue" ontouchstart="startMove('backward')" ontouchend="stopMove()">GERİ</button>
        <button class="btn c-orange" ontouchstart="startMove('lean_r')" ontouchend="stopMove()">SAĞ<br>EĞİL</button>
    </div>
    <div class="btn-row">
        <button class="btn c-grey" onclick="sendCommand('height_up')">YÜKSEL</button>
        <button id="btnAuto" class="btn c-teal big-btn" onclick="toggleAuto()">OTONOM</button>
        <button class="btn c-grey" onclick="sendCommand('height_down')">ALÇAL</button>
    </div>
    <script>
        function sendCommand(cmd) { fetch('/cmd/' + cmd); }
        function startMove(dir) { fetch('/move/' + dir); }
        function stopMove() { fetch('/move/stop'); }
        function startTracking() {
            let target = prompt("Hangi nesneyi takip edeyim?");
            if (target) {
                fetch('/start_tracking', {
                    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({target: target})
                }).then(r => r.json()).then(d => { if(d.status != "ok") alert("Hata: " + d.msg); });
            }
        }
        function toggleTrot() { fetch('/toggle/trot').then(r=>r.json()).then(d=>{ let b=document.getElementById("btnTrot"); if(d.status){b.innerText="TROT\\nDURDUR";b.className="btn c-green big-btn"}else{b.innerText="TROT\\nBAŞLAT";b.className="btn c-red big-btn"} }); }
        function toggleBalance() { fetch('/toggle/balance').then(r=>r.json()).then(d=>{ document.getElementById("btnBal").className=d.status?"btn c-green":"btn c-grey"; }); }
        function toggleAuto() { fetch('/toggle/auto').then(r=>r.json()).then(d=>{ let b=document.getElementById("btnAuto"); if(d.status){b.innerText="OTONOM\\nAÇIK";b.className="btn c-green big-btn"}else{b.innerText="OTONOM";b.className="btn c-teal big-btn"} }); }
        
        function toggleApproach() {
            fetch('/toggle/approach').then(r => r.json()).then(d => {
                let btn = document.getElementById("btnApproach");
                if (d.status) { btn.innerText = "GİDİLİYOR..."; btn.style.background = "#4CAF50"; }
                else { btn.innerText = "🚀 HEDEFE GİT"; btn.style.background = "#2196F3"; }
            });
        }
        function toggleMic() { fetch('/toggle/mic').then(r=>r.json()).then(d=>{ let b=document.getElementById("btnMic"); if(d.status){b.innerText="DİNLİYOR";b.className="btn c-red"}else{b.innerText="MİKROFON";b.className="btn c-orange"} }); }
        function sendText() { let i=document.getElementById("chatInput"); let t=i.value.trim(); if(t===""){return;} i.value=""; fetch('/send_text', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({text:t})}); }
        function handleKeyPress(e) { if(e.key==='Enter') sendText(); }
        
        setInterval(() => {
            fetch('/get_status').then(res => res.json()).then(data => {
                let box = document.getElementById("logBox");
                let html = ""; data.logs.forEach(l => { html += l + "<br>"; });
                if (box.innerHTML !== html) { box.innerHTML = html; box.scrollTop = box.scrollHeight; 
                    if(!data.mic_active && document.getElementById("btnMic").innerText!=="MİKROFON") {
                         document.getElementById("btnMic").innerText="MİKROFON"; document.getElementById("btnMic").className="btn c-grey";
                    }
                }
                
                let trkBtn = document.getElementById("btnTrack");
                let appBtn = document.getElementById("btnApproach");
                if (data.tracking) { 
                    trkBtn.innerText = "🛑 TAKİBİ DURDUR"; 
                    trkBtn.style.background = "red"; 
                    trkBtn.onclick = function() { sendCommand('stop_tracking'); };
                    appBtn.style.display = "block";
                    if(data.approach) { appBtn.innerText = "GİDİLİYOR..."; appBtn.style.background = "#4CAF50"; } 
                    else { appBtn.innerText = "🚀 HEDEFE GİT"; appBtn.style.background = "#2196F3"; }
                } else { 
                    trkBtn.innerText = "🎯 HEDEF TAKİP BAŞLAT"; 
                    trkBtn.style.background = "#E91E63"; 
                    trkBtn.onclick = startTracking;
                    appBtn.style.display = "none";
                }
            });
        }, 500);
    </script>
</body>
</html>
"""

# ==================================================================
# 6. FLASK ROUTE'LARI
# ==================================================================
@app.route('/')
def index(): return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/send_text', methods=['POST'])
def send_text():
    data = request.json; text = data.get('text', '')
    if text: threading.Thread(target=process_text_thread, args=(text,), daemon=True).start()
    return jsonify(status="ok")

@app.route('/start_tracking', methods=['POST'])
def start_tracking():
    data = request.json; target = data.get('target', '')
    if not target or last_frame_cv2 is None: return jsonify(status="error", msg="Kamera yok")
    
    obstacle_avoider.stop()
    robot_state["auto"] = False
    
    if object_tracker.init_tracking(last_frame_cv2, target):
        robot_state["tracking"] = True
        robot_state["target_name"] = target
        add_log("Takip", f"{target} kilitlendi!")
        return jsonify(status="ok")
    else:
        return jsonify(status="error", msg="Nesne bulunamadı")

@app.route('/cmd/<action>')
def command(action):
    if action == 'cancel_mic':
        global rec_process
        if rec_process: rec_process.terminate()
        stop_audio_playback()
        robot_state["mic"] = False
        add_log("Sistem", "Kayıt İptal.")
    elif action == 'stop_tracking':
        object_tracker.stop()
        robot_state["tracking"] = False
        add_log("Takip", "Durduruldu.")
    elif action == 'height_up':
        try: h = locomotion._height_factor / 40.0; locomotion.set_height_offset(min(1.0, h + 0.05))
        except: pass
    elif action == 'height_down':
        try: h = locomotion._height_factor / 40.0; locomotion.set_height_offset(max(0.0, h - 0.05))
        except: pass
    return "ok"

@app.route('/move/<direction>')
def move(direction):
    if robot_state["auto"] or robot_state["tracking"]: return "auto_mode_on"
    if not robot_state["trot"] and direction not in ['lean_l', 'lean_r', 'stop']: return "trot_off"
    if direction == 'stop': locomotion.set_forward_factor(0.0); locomotion.set_rotation_factor(0.0); locomotion.set_strafe_factor(0.0); locomotion.set_lean(0.0)
    elif direction == 'forward': locomotion.set_forward_factor(1.0)
    elif direction == 'backward': locomotion.set_forward_factor(-1.0)
    elif direction == 'turn_l': locomotion.set_rotation_factor(-1.0)
    elif direction == 'turn_r': locomotion.set_rotation_factor(1.0)
    elif direction == 'strafe_l': locomotion.set_strafe_factor(-1.0)
    elif direction == 'strafe_r': locomotion.set_strafe_factor(1.0)
    elif direction == 'lean_l': locomotion.set_lean(-1.0)
    elif direction == 'lean_r': locomotion.set_lean(1.0)
    return "ok"

@app.route('/toggle/<feature>')
def toggle(feature):
    if feature == 'trot':
        locomotion.toggle_standing()
        robot_state["trot"] = not robot_state["trot"]
        return jsonify(status=robot_state["trot"])
    elif feature == 'balance':
        robot_state["balance"] = not robot_state["balance"]
        locomotion.set_balance_mode(robot_state["balance"])
        return jsonify(status=robot_state["balance"])
    elif feature == 'auto':
        robot_state["auto"] = not robot_state["auto"]
        if robot_state["auto"]: obstacle_avoider.start(); add_log("Sistem", "🚀 Otonom Açık.")
        else: obstacle_avoider.stop(); add_log("Sistem", "🛑 Otonom Kapalı.")
        return jsonify(status=robot_state["auto"])
    
    elif feature == 'approach':
        if robot_state["tracking"]:
            robot_state["approach"] = not robot_state["approach"]
            if robot_state["approach"]: add_log("Takip", "Hedefe gidiliyor...")
            else: 
                add_log("Takip", "İlerleme durdu.")
                locomotion.set_forward_factor(0.0)
            return jsonify(status=robot_state["approach"])
        else:
            return jsonify(status=False)

    elif feature == 'mic':
        global rec_process
        if not robot_state["mic"]:
            stop_audio_playback()
            robot_state["mic"] = True
            locomotion.set_forward_factor(0.0)
            rec_process = subprocess.Popen(["arecord", "-D", "plughw:3,0", "-f", "cd", "-t", "wav", AUDIO_FILE], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            robot_state["mic"] = False
            if rec_process: rec_process.terminate()
            threading.Thread(target=process_audio_thread).start()
        return jsonify(status=robot_state["mic"])

@app.route('/get_status')
def get_status():
    return jsonify(
        logs=robot_state["log"], 
        mic_active=robot_state["mic"], 
        tracking=robot_state["tracking"], 
        approach=robot_state["approach"], 
        photo_time=robot_state["last_photo_time"]
    )

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)