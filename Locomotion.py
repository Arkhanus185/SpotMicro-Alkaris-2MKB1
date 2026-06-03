import time
import math
import asyncio
from enum import Enum

from InverseKinematics import InverseKinematics, interpolate, KeyframeKinematics
from MPU6050 import MPU6050
import numpy as np

class Locomotion:
    """Main class for running keyframe animation for SpotMicro."""
    class Leg(Enum):
        FrontLeft = 0
        FrontRight = 1
        BackLeft = 2
        BackRight = 3
    
    def __init__(self, servos):
        self._servos = servos
        self._running = False
        self._keyframes = []
        for i in range(4):
            self._keyframes.append([[0, 150, 0], [0, 150, 0]])
            
        self._standing = True
        self._forward_factor = 0.0
        self._rotation_factor = 0.0
        self._lean_factor = 0.0
        self._height_factor = 0.0
        self._strafe_factor = 0.0
        
        # Denge modu
        self._balance_mode = False
        self._mpu = None
        self._balance_calibration = None
        self._height_adjustments = {
            self.Leg.FrontLeft: 0.0,
            self.Leg.FrontRight: 0.0,
            self.Leg.BackLeft: 0.0,
            self.Leg.BackRight: 0.0
        }
        self._pid_gains = {'kp': 1.0, 'ki': 0, 'kd': 0}
        self._pid_errors = {'pitch': {'integral': 0, 'last_error': 0},
                           'roll': {'integral': 0, 'last_error': 0}}
        self._max_height_adjustment = 25
        self._last_time = time.time()

    def init_balance_mode(self):
        try:
            self._mpu = MPU6050()
            config_file = "denge_config.csv"
            try:
                calibration_data = np.loadtxt(config_file, delimiter=",")
                self._balance_calibration = {
                    'accel_offset': calibration_data[0:3].tolist(),
                    'gyro_offset': calibration_data[3:6].tolist(),
                    'target_pitch': calibration_data[6],
                    'target_roll': calibration_data[7]
                }
                self._mpu.load_calibration(self._balance_calibration)
                print(f"✅ Denge kalibrasyonu yüklendi: Hedef Pitch={self._balance_calibration['target_pitch']:.2f}°, Roll={self._balance_calibration['target_roll']:.2f}°")
            except FileNotFoundError:
                print("⚠️  Kalibrasyon dosyası bulunamadı! Önce denge_kalibrasyon.py çalıştırın.")
                self._balance_mode = False
                return False
            return True
        except Exception as e:
            print(f"❌ MPU6050 başlatılamadı: {e}")
            self._balance_mode = False
            return False

    def set_balance_mode(self, enabled):
        if enabled and self._mpu is None:
            if not self.init_balance_mode():
                return False
        self._balance_mode = enabled
        print(f"🔄 Denge modu: {'Açık' if enabled else 'Kapalı'}")
        return True

    def _pid_control(self, error, axis):
        current_time = time.time()
        dt = current_time - self._last_time
        self._last_time = current_time

        pid = self._pid_errors[axis]
        pid['integral'] += error * dt
        derivative = (error - pid['last_error']) / dt if dt > 0 else 0
        output = (self._pid_gains['kp'] * error + 
                  self._pid_gains['ki'] * pid['integral'] + 
                  self._pid_gains['kd'] * derivative)
        pid['last_error'] = error
        
        return max(-self._max_height_adjustment, 
                   min(output, self._max_height_adjustment))

    def _calculate_height_adjustments(self):
        if not self._balance_mode or self._mpu is None:
            return {leg: 0 for leg in self._height_adjustments}

        pitch_error, roll_error = self._mpu.get_tilt_angles(
            self._balance_calibration['target_pitch'],
            self._balance_calibration['target_roll']
        )

        pitch_adjustment = -self._pid_control(pitch_error, 'pitch')
        roll_adjustment = -self._pid_control(roll_error, 'roll')

        adjustments = {}
        front_adj = -pitch_adjustment
        back_adj = pitch_adjustment
        right_adj = roll_adjustment    # Tersine çevrildi
        left_adj = -roll_adjustment    # Tersine çevrildi

        adjustments[self.Leg.FrontLeft] = front_adj + left_adj
        adjustments[self.Leg.FrontRight] = front_adj + right_adj
        adjustments[self.Leg.BackLeft] = back_adj + left_adj
        adjustments[self.Leg.BackRight] = back_adj + right_adj

        self._height_adjustments = adjustments
        return adjustments

    async def Run(self):
        elapsed = 0.0
        forward_gait = [[-10.0, 150.0, 40.0], [-10.0, 120.0, 40.0], [10.0, 120.0, 40.0], 
                        [10.0, 150.0, 40.0], [3.5, 150.0, 40.0], [-3.5, 150, 40]]
        strafe_gait = [
            [0, 120, 40], [0, 120, 50], [0, 150, 50], [0, 150, 40],
            [0, 120, 40], [0, 120, 30], [0, 150, 30], [0, 150, 40],
            [0, 150, 43.5], [0, 150, 36.5]
        ]
        start = time.time()
        
        last_index = -1
        last_keyframes = None
        self._running = True
        while self._running:
            elapsed += (time.time() - start) * 15
            gait = strafe_gait if self._strafe_factor != 0 else forward_gait
            if elapsed >= len(gait):
                elapsed -= len(gait)
            start = time.time()
            index = math.floor(elapsed)
            ratio = elapsed - index

            if last_index != index:
                self._shift_keyframes()
                
                if self._standing:
                    self._set_standing_keyframes()
                else:
                    if self._strafe_factor != 0:
                        if self._strafe_factor < 0:
                            if index < 4:
                                self._keyframes[self.Leg.FrontRight.value][1] = [0, strafe_gait[index][1], strafe_gait[index][2]]
                                self._keyframes[self.Leg.BackLeft.value][1] = [0, strafe_gait[index][1], 80 - strafe_gait[index][2]]
                                self._keyframes[self.Leg.FrontLeft.value][1] = [0, 150, 40]
                                self._keyframes[self.Leg.BackRight.value][1] = [0, 150, 40]
                            else:
                                self._keyframes[self.Leg.FrontLeft.value][1] = [0, strafe_gait[index][1], 80 - strafe_gait[index][2]]
                                self._keyframes[self.Leg.BackRight.value][1] = [0, strafe_gait[index][1], strafe_gait[index][2]]
                                self._keyframes[self.Leg.FrontRight.value][1] = [0, 150, 40]
                                self._keyframes[self.Leg.BackLeft.value][1] = [0, 150, 40]
                        else:
                            if index < 4:
                                self._keyframes[self.Leg.FrontRight.value][1] = [0, strafe_gait[index][1], 80 - strafe_gait[index][2]]
                                self._keyframes[self.Leg.BackLeft.value][1] = [0, strafe_gait[index][1], strafe_gait[index][2]]
                                self._keyframes[self.Leg.FrontLeft.value][1] = [0, 150, 40]
                                self._keyframes[self.Leg.BackRight.value][1] = [0, 150, 40]
                            else:
                                self._keyframes[self.Leg.FrontLeft.value][1] = [0, strafe_gait[index][1], strafe_gait[index][2]]
                                self._keyframes[self.Leg.BackRight.value][1] = [0, strafe_gait[index][1], 80 - strafe_gait[index][2]]
                                self._keyframes[self.Leg.FrontRight.value][1] = [0, 150, 40]
                                self._keyframes[self.Leg.BackLeft.value][1] = [0, 150, 40]
                    else:
                        angle = 45.0 / 180.0 * math.pi
                        x_rot = math.sin(angle) * self._rotation_factor
                        z_rot = math.cos(angle) * self._rotation_factor
                        angle = (45 + gait[index][0]) / 180.0 * math.pi
                        x_rot = x_rot - math.sin(angle) * self._rotation_factor
                        z_rot = z_rot - math.cos(angle) * self._rotation_factor
                        self._keyframes[self.Leg.FrontRight.value][1] = [gait[index][0] * self._forward_factor + x_rot, 
                                                                        gait[index][1], gait[index][2] + z_rot]
                        self._keyframes[self.Leg.BackLeft.value][1] = [gait[index][0] * self._forward_factor - x_rot, 
                                                                      gait[index][1], gait[index][2] + z_rot]
                        adjusted_index = index + 3
                        if adjusted_index >= len(gait): adjusted_index -= len(gait)
                        angle = 45.0 / 180.0 * math.pi
                        x_rot = math.sin(angle) * self._rotation_factor
                        z_rot = math.cos(angle) * self._rotation_factor
                        angle = (45 + gait[adjusted_index][0]) / 180.0 * math.pi
                        x_rot = x_rot - math.sin(angle) * self._rotation_factor
                        z_rot = z_rot - math.cos(angle) * self._rotation_factor
                        self._keyframes[self.Leg.FrontLeft.value][1] = [gait[adjusted_index][0] * self._forward_factor - x_rot, 
                                                                       gait[adjusted_index][1], gait[adjusted_index][2] - z_rot]
                        self._keyframes[self.Leg.BackRight.value][1] = [gait[adjusted_index][0] * self._forward_factor + x_rot, 
                                                                       gait[adjusted_index][1], gait[adjusted_index][2] - z_rot]

                last_index = index

            current_keyframes = (
                tuple(self._keyframes[self.Leg.FrontRight.value][1]),
                tuple(self._keyframes[self.Leg.BackLeft.value][1]),
                tuple(self._keyframes[self.Leg.FrontLeft.value][1]),
                tuple(self._keyframes[self.Leg.BackRight.value][1])
            )
            if last_keyframes != current_keyframes:
                print(f"Keyframes: FR={current_keyframes[0]}, BL={current_keyframes[1]}, FL={current_keyframes[2]}, BR={current_keyframes[3]}")
                last_keyframes = current_keyframes

            self._InterpolateKeyframes(ratio)

            await asyncio.sleep(0)

        self._shift_keyframes(elapsed - math.floor(elapsed))
        self._set_standing_keyframes()
        self._standing = True
        elapsed = 0.0
        start = time.time()
        while elapsed < 1:
            elapsed += (time.time() - start) * 20
            start = time.time()
            self._InterpolateKeyframes(elapsed)
            await asyncio.sleep(0)

    def set_forward_factor(self, factor):
        self._forward_factor = factor * 2

    def set_rotation_factor(self, factor):
        self._rotation_factor = factor * 100

    def set_lean(self, lean):
        self._lean_factor = lean * 20

    def set_height_offset(self, height):
        self._height_factor = height * 40

    def set_strafe_factor(self, factor):
        self._strafe_factor = factor

    def toggle_standing(self):
        self._standing = not self._standing

    def Shutdown(self):
        self._running = False

    def _InterpolateKeyframes(self, ratio):
        balance_adjustments = self._calculate_height_adjustments() if self._balance_mode else {leg: 0 for leg in self._height_adjustments}
        
        keyframes = self._keyframes[self.Leg.FrontRight.value]
        foot, leg, shoulder = KeyframeKinematics(
            [keyframes[0][0], keyframes[0][1] + self._height_factor + balance_adjustments[self.Leg.FrontRight], keyframes[0][2] - self._lean_factor],
            [keyframes[1][0], keyframes[1][1] + self._height_factor + balance_adjustments[self.Leg.FrontRight], keyframes[1][2] - self._lean_factor],
            ratio
        )
        self._FrontRightLeg(foot, leg, shoulder)

        keyframes = self._keyframes[self.Leg.BackLeft.value]
        foot, leg, shoulder = KeyframeKinematics(
            [keyframes[0][0], keyframes[0][1] + self._height_factor + balance_adjustments[self.Leg.BackLeft], keyframes[0][2] + self._lean_factor],
            [keyframes[1][0], keyframes[1][1] + self._height_factor + balance_adjustments[self.Leg.BackLeft], keyframes[1][2] + self._lean_factor],
            ratio
        )
        self._BackLeftLeg(foot, leg, shoulder)

        keyframes = self._keyframes[self.Leg.FrontLeft.value]
        foot, leg, shoulder = KeyframeKinematics(
            [keyframes[0][0], keyframes[0][1] + self._height_factor + balance_adjustments[self.Leg.FrontLeft], keyframes[0][2] + self._lean_factor],
            [keyframes[1][0], keyframes[1][1] + self._height_factor + balance_adjustments[self.Leg.FrontLeft], keyframes[1][2] + self._lean_factor],
            ratio
        )
        self._FrontLeftLeg(foot, leg, shoulder)
        
        keyframes = self._keyframes[self.Leg.BackRight.value]
        foot, leg, shoulder = KeyframeKinematics(
            [keyframes[0][0], keyframes[0][1] + self._height_factor + balance_adjustments[self.Leg.BackRight], keyframes[0][2] - self._lean_factor],
            [keyframes[1][0], keyframes[1][1] + self._height_factor + balance_adjustments[self.Leg.BackRight], keyframes[1][2] - self._lean_factor],
            ratio
        )
        self._BackRightLeg(foot, leg, shoulder)
    
    def _FrontRightLeg(self, foot, leg, shoulder):
        self._servos[2].set_angle(180 - foot)
        self._servos[1].set_angle(180 - (leg + 90))
        self._servos[0].set_angle(shoulder)
    
    def _FrontLeftLeg(self, foot, leg, shoulder):
        self._servos[5].set_angle(foot)
        self._servos[4].set_angle(180 - (90 - leg))
        self._servos[3].set_angle(180 - shoulder)
    
    def _BackRightLeg(self, foot, leg, shoulder):
        self._servos[8].set_angle(180 - foot)
        self._servos[7].set_angle(180 - (leg + 90))
        self._servos[6].set_angle(shoulder)
    
    def _BackLeftLeg(self, foot, leg, shoulder):
        self._servos[11].set_angle(foot)
        self._servos[10].set_angle(180 - (90 - leg))
        self._servos[9].set_angle(180 - shoulder)

    def _shift_keyframes(self, ratio=0):
        if ratio > 0:
            for i in range(len(self._keyframes)):
                key1 = self._keyframes[i][0]
                key2 = self._keyframes[i][1]
                self._keyframes[i][0] = [interpolate(key1[0], key2[0], ratio), 
                                        interpolate(key1[1], key2[1], ratio), 
                                        interpolate(key1[2], key2[2], ratio)]
        else:
            for i in range(len(self._keyframes)):
                self._keyframes[i][0] = self._keyframes[i][1]

    def _set_standing_keyframes(self):
        for i in range(4):
            self._keyframes[i][1] = [0, 150, 40]