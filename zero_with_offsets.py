import smbus, time
import numpy as np
import argparse

bus = smbus.SMBus(1)

def initialize(addr):
    bus.write_byte_data(addr, 0, 0x20)
    time.sleep(.25)
    bus.write_byte_data(addr, 0, 0x10)
    time.sleep(.25)
    bus.write_byte_data(addr, 0xfe, 0x79)
    bus.write_byte_data(addr, 0, 0x20)
    time.sleep(.25)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set servos to zero with offsets")
    parser.add_argument('--address', type=lambda x: int(x, 0), default=0x40, help="PCA9685 I2C address")
    parser.add_argument('--offset_start', type=int, default=0, help="Offsets dizisinden başlama indeksi (ön için 0, arka için 6)")
    
    args = parser.parse_args()
    addr = args.address
    offset_start = args.offset_start
    
    initialize(addr)
    
    ports = [0, 1, 2, 13, 14, 15]
    
    offsets = np.loadtxt("ENTER HERE !!! config.csv file path", delimiter=",", dtype=float)  # Enter config.csv file path
    offsets = offsets[offset_start:offset_start+6] 
    
    for p in ports:
        port = p * 4 + 0x06
        bus.write_word_data(addr, port, 0)

    time.sleep(.25)
    for i in range(len(ports)):
        port = ports[i] * 4 + 0x06
        value = int(312 + offsets[i])  
        bus.write_word_data(addr, port + 0x02, value)