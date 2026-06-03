import smbus
import math
import time
import numpy as np

class MPU6050:
    """MPU6050 denge sensörü için sınıf.
    
    Bu sınıf MPU6050 sensöründen açı verilerini okur ve 
    kalibrasyon değerlerini yönetir.
    """
    
    # MPU6050 Register adresleri
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47

    def __init__(self, bus=1, address=0x68):
        """MPU6050 sensörünü başlat.
        
        Parameters
        ----------
        bus : int
            I2C bus numarası (Raspberry Pi 4 için genelde 1)
        address : int
            MPU6050'nin I2C adresi (varsayılan 0x68)
        """
        self.bus = smbus.SMBus(bus)
        self.address = address
        
        # Sensörü uyandır
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
        time.sleep(0.1)
        
        # Örnekleme hızını ayarla (1kHz / (1 + 7) = 125Hz)
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 7)
        
        # Düşük geçiren filtre (DLPF) ayarı
        self.bus.write_byte_data(self.address, self.CONFIG, 0)
        
        # Gyro hassasiyeti (±250 deg/s)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0)
        
        # Accelerometer hassasiyeti (±2g)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0)
        
        # Kalibrasyon değerleri
        self.accel_offset = [0, 0, 0]
        self.gyro_offset = [0, 0, 0]
        
        # Complementary filter için değerler
        self.pitch = 0
        self.roll = 0
        self.last_time = time.time()
        
    def read_raw_data(self, addr):
        """Ham veriyi oku ve işaretle 16-bit değere dönüştür."""
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = ((high << 8) | low)
        
        if value > 32768:
            value = value - 65536
        return value

    def get_accel_data(self):
        """Accelerometer verilerini oku (g cinsinden)."""
        acc_x = -self.read_raw_data(self.ACCEL_XOUT_H) / 16384.0  # Ters çevirme (sensör yönlendirme)
        acc_y = -self.read_raw_data(self.ACCEL_YOUT_H) / 16384.0  # Ters çevirme (sensör yönlendirme)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H) / 16384.0
        return acc_x - self.accel_offset[0], \
               acc_y - self.accel_offset[1], \
               acc_z - self.accel_offset[2]

    def get_gyro_data(self):
        """Gyroscope verilerini oku (derece/saniye cinsinden)."""
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H) / 131.0
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H) / 131.0
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H) / 131.0
        return gyro_x - self.gyro_offset[0], \
               gyro_y - self.gyro_offset[1], \
               gyro_z - self.gyro_offset[2]

    def get_angles(self):
        """Complementary filter kullanarak pitch ve roll açılarını hesapla.
        
        Returns
        -------
        tuple : (pitch, roll) derece cinsinden
        """
        # Accelerometer'dan açıları hesapla
        acc_x, acc_y, acc_z = self.get_accel_data()
        
        # Roll ve pitch açılarını hesapla (radyan)
        acc_pitch = math.atan2(acc_x, math.sqrt(acc_y*acc_y + acc_z*acc_z))
        acc_roll = math.atan2(acc_y, math.sqrt(acc_x*acc_x + acc_z*acc_z))
        
        # Dereceye çevir
        acc_pitch_deg = acc_pitch * 180 / math.pi
        acc_roll_deg = acc_roll * 180 / math.pi
        
        # Gyroscope verilerini al
        gyro_x, gyro_y, gyro_z = self.get_gyro_data()
        
        # Zaman farkını hesapla
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Complementary filter (0.98 gyro, 0.02 accelerometer)
        self.pitch = 0.98 * (self.pitch + gyro_x * dt) + 0.02 * acc_pitch_deg
        self.roll = 0.98 * (self.roll + gyro_y * dt) + 0.02 * acc_roll_deg
        
        return self.pitch, self.roll

    def calibrate(self, samples=100):
        """Sensörü kalibre et.
        
        Parameters
        ----------
        samples : int
            Kalibrasyon için alınacak örnek sayısı
            
        Returns
        -------
        dict : Kalibrasyon değerleri
        """
        print(f"Kalibrasyon başlıyor... {samples} örnek alınacak")
        print("Lütfen robotu düz bir zemine koyun ve hareket ettirmeyin!")
        
        accel_sum = [0, 0, 0]
        gyro_sum = [0, 0, 0]
        
        # İlk birkaç okumayı at (sensör stabilizasyonu için)
        for _ in range(10):
            self.get_accel_data()
            self.get_gyro_data()
            time.sleep(0.01)
        
        # Örnekleri topla
        for i in range(samples):
            acc_x = self.read_raw_data(self.ACCEL_XOUT_H) / 16384.0
            acc_y = self.read_raw_data(self.ACCEL_YOUT_H) / 16384.0
            acc_z = self.read_raw_data(self.ACCEL_ZOUT_H) / 16384.0
            
            gyro_x = self.read_raw_data(self.GYRO_XOUT_H) / 131.0
            gyro_y = self.read_raw_data(self.GYRO_YOUT_H) / 131.0
            gyro_z = self.read_raw_data(self.GYRO_ZOUT_H) / 131.0
            
            accel_sum[0] += acc_x
            accel_sum[1] += acc_y
            accel_sum[2] += acc_z
            
            gyro_sum[0] += gyro_x
            gyro_sum[1] += gyro_y
            gyro_sum[2] += gyro_z
            
            if (i + 1) % 20 == 0:
                print(f"İlerleme: {i+1}/{samples}")
            
            time.sleep(0.01)
        
        # Ortalama offset değerlerini hesapla
        self.accel_offset[0] = accel_sum[0] / samples
        self.accel_offset[1] = accel_sum[1] / samples
        self.accel_offset[2] = (accel_sum[2] / samples) - 1.0  # Yerçekimi için düzeltme
        
        self.gyro_offset[0] = gyro_sum[0] / samples
        self.gyro_offset[1] = gyro_sum[1] / samples
        self.gyro_offset[2] = gyro_sum[2] / samples
        
        print("Kalibrasyon tamamlandı!")
        
        return {
            'accel_offset': self.accel_offset.copy(),
            'gyro_offset': self.gyro_offset.copy(),
            'target_pitch': self.pitch,
            'target_roll': self.roll
        }

    def load_calibration(self, calibration_data):
        """Kalibrasyon verilerini yükle.
        
        Parameters
        ----------
        calibration_data : dict
            Kalibrasyon değerleri
        """
        self.accel_offset = calibration_data['accel_offset']
        self.gyro_offset = calibration_data['gyro_offset']
        
    def get_tilt_angles(self, target_pitch=0, target_roll=0):
        """Hedef açılardan sapma miktarını hesapla.
        
        Parameters
        ----------
        target_pitch : float
            Hedef pitch açısı (derece)
        target_roll : float
            Hedef roll açısı (derece)
            
        Returns
        -------
        tuple : (pitch_error, roll_error) derece cinsinden sapma
        """
        current_pitch, current_roll = self.get_angles()
        
        pitch_error = current_pitch - target_pitch
        roll_error = current_roll - target_roll
        
        return pitch_error, roll_error