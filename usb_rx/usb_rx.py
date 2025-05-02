import serial
import ctypes
import struct
import numpy as np
from threading import Thread
import crc16

class serial_node(serial.Serial):
    def __init__(self):
        super().__init__('COM6',115200)
        self.crc=crc16.crc16()
        self.s=struct.Struct('=B3f5f5fH')
        self.rx_buffer=ctypes.create_string_buffer(self.s.size)
        self.rx_data=np.zeros(1, dtype=self.rx_bag)

    rx_bag=np.dtype({
        'names':['header','imu_rpy','light','electric','crc_sum'],
        'formats':['B','3f','5f','5f','H']
    })

    def serial_open(self):
        while True:
            if self.is_open:
                break
            else:
                try:
                    self.open()
                    print("Serial open success.")
                    break
                except serial.SerialException:
                    None

    def serial_restart(self):
        try:
            self.close()
            self.open()
            print("Serial reset success.")
        except:
            None

    def receive(self):
        while True:
            try:
                count = self.inWaiting()
                if count > 0:
                # if self.inWaiting()==self.s.size:
                    self.rx_buffer=self.read(count)
                    if self.rx_buffer[0]==0x5A and self.crc.verify(self.rx_buffer,self.s.size):
                        self.unpacked_data = self.s.unpack_from(self.rx_buffer,0)
                        self.rx_data['header']=self.unpacked_data[0]
                        self.rx_data['imu_rpy']=self.unpacked_data[1]
                        self.rx_data['light']=self.unpacked_data[2]
                        self.rx_data['electric']=self.unpacked_data[3]
                        self.rx_data['crc_sum']=self.unpacked_data[4]
                        print(self.rx_data)

                else:
                    None
            except serial.SerialException:
                None
            except:
                self.serial_restart()

def main():
    node=serial_node()
    node.serial_open()

    executor_thread = Thread(target=node.receive, daemon=True, args=())
    executor_thread.start()
    executor_thread.join()

if __name__ == "__main__":
 	main()
