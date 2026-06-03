import smbus, time
import argparse

bus = smbus.SMBus(1)

def initialize(addr):
    bus.write_byte_data(addr, 0, 0x20)  # enables word writes
    time.sleep(.25)
    bus.write_byte_data(addr, 0, 0x10)  # enable Prescale change
    time.sleep(.25)
    bus.write_byte_data(addr, 0xfe, 0x79)  # 50 Hz prescale
    bus.write_byte_data(addr, 0, 0x20)  # enables word writes
    time.sleep(.25)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set all servos to zero")
    parser.add_argument('--address', type=lambda x: int(x, 0), default=0x40, help="PCA9685 I2C address (hex, e.g. 0x40)")
    
    args = parser.parse_args()
    addr = args.address
    
    initialize(addr)
    
    ports = [0, 1, 2, 13, 14, 15]  # Pins of PCA9685

    for p in ports:
        port = p * 4 + 0x06
        bus.write_word_data(addr, port, 0)  # start time = 0us

    time.sleep(.25)
    for p in ports:
        port = p * 4 + 0x06
        bus.write_word_data(addr, port + 0x02, 312)  # end time = 2.0ms (90 degrees)