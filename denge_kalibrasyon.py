import numpy as np
from MPU6050 import MPU6050

def calibrate_mpu6050():
    """MPU6050 sensörünü kalibre et ve verileri kaydet."""
    mpu = MPU6050()
    
    print("MPU6050 kalibrasyonu başlıyor...")
    calibration_data = mpu.calibrate(samples=200)
    
    # Verileri birleştir
    data_to_save = np.array([
        calibration_data['accel_offset'][0], calibration_data['accel_offset'][1], calibration_data['accel_offset'][2],
        calibration_data['gyro_offset'][0], calibration_data['gyro_offset'][1], calibration_data['gyro_offset'][2],
        calibration_data['target_pitch'], calibration_data['target_roll']
    ])
    
    # Dosyaya kaydet
    file_path = "denge_config.csv" # denge_config.csv dosyasını oluştur ve yolunu buraya gir
    np.savetxt(file_path, [data_to_save], delimiter=",")
    print(f"Kalibrasyon verileri {file_path} dosyasına kaydedildi!")

if __name__ == "__main__":
    calibrate_mpu6050()