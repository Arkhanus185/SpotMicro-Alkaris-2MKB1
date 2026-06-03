import RPi.GPIO as GPIO
import time
import threading
import random

class ObstacleAvoidance:
    def __init__(self, locomotion_obj):
        self.locomotion = locomotion_obj
        self.active = False
        self.thread = None
        
        # Pin Tanımlamaları
        self.LEFT_TRIG = 23
        self.LEFT_ECHO = 24
        self.RIGHT_TRIG = 5
        self.RIGHT_ECHO = 6
        
        # Ayarlar
        self.SAFE_DISTANCE = 40.0 # cm
        self.AVOID_TIME = 4.0     # saniye

        # GPIO Kurulumu
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.LEFT_TRIG, GPIO.OUT)
            GPIO.setup(self.LEFT_ECHO, GPIO.IN)
            GPIO.setup(self.RIGHT_TRIG, GPIO.OUT)
            GPIO.setup(self.RIGHT_ECHO, GPIO.IN)
        except Exception as e:
            print(f"⚠️ Mesafe sensörü hatası: {e}")

    def get_distance(self, trig, echo):
        try:
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)

            start = time.time()
            stop = time.time()
            
            # Zaman aşımı kontrolü (sensör kilitlenmesin)
            timeout = time.time() + 0.04

            while GPIO.input(echo) == 0:
                start = time.time()
                if start > timeout: return 100.0

            while GPIO.input(echo) == 1:
                stop = time.time()
                if stop > timeout: return 100.0

            distance = ((stop - start) * 34300) / 2
            return distance
        except:
            return 100.0

    def start(self):
        if not self.active:
            self.active = True
            # Robot oturyorsa kaldır (Ana koddan state kontrolü yapılmalı ama buraya da ekledik)
            if not self.locomotion._standing:
                self.locomotion.toggle_standing()
            
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()
            print("🚀 Otonom Mod Başlatıldı.")

    def stop(self):
        self.active = False
        if self.thread:
            self.thread.join(timeout=1)
        self.locomotion.set_forward_factor(0.0)
        self.locomotion.set_rotation_factor(0.0)
        print("🛑 Otonom Mod Durduruldu.")

    def _run_loop(self):
        time.sleep(1) # Başlamadan önce bekle
        
        while self.active:
            # 1. Ölçüm
            d_left = self.get_distance(self.LEFT_TRIG, self.LEFT_ECHO)
            time.sleep(0.01) # Girişim önleme
            d_right = self.get_distance(self.RIGHT_TRIG, self.RIGHT_ECHO)
            
            # 2. Karar
            if d_left < self.SAFE_DISTANCE and d_right < self.SAFE_DISTANCE:
                # Blokaj: Rastgele dön
                direction = random.choice([-1.0, 1.0])
                self.locomotion.set_forward_factor(0.0)
                self.locomotion.set_rotation_factor(direction)
                time.sleep(self.AVOID_TIME)
            
            elif d_left < self.SAFE_DISTANCE:
                # Engel Sağda (Sol sensör görüyor) -> Sola dön
                self.locomotion.set_forward_factor(0.0)
                self.locomotion.set_rotation_factor(-1.0)
                time.sleep(self.AVOID_TIME)
                
            elif d_right < self.SAFE_DISTANCE:
                # Engel Solda (Sağ sensör görüyor) -> Sağa dön
                self.locomotion.set_forward_factor(0.0)
                self.locomotion.set_rotation_factor(1.0)
                time.sleep(self.AVOID_TIME)
                
            else:
                # Temiz -> İleri
                self.locomotion.set_rotation_factor(0.0)
                self.locomotion.set_forward_factor(1.0)
            
            time.sleep(0.1)