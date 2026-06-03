import smbus, time
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
    parser = argparse.ArgumentParser(description="Tool to measure servo position offsets")
    parser.add_argument('port', type=int, help="Servo pin (e.g. 0,1,2,13,14,15)")
    parser.add_argument('--address', type=lambda x: int(x, 0), default=0x40, help="PCA9685 I2C address")
    
    args = parser.parse_args()
    port = args.port * 4 + 0x06
    addr = args.address
    
    initialize(addr)
    
    bus.write_word_data(addr, port, 0)
    time.sleep(2)
    
    value = 312
    bus.write_word_data(addr, port + 0x02, value)
    
    print("Adjust center position")
    print("Enter amount to adjust or leave blank to exit")
    
    done = False
    while not done:
        text = input("adjustment: ")
        if text == "":
            done = True
        else:
            value += int(text)
            bus.write_word_data(addr, port + 0x02, value)
    
    print("Finished: ", (value - 312))