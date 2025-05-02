import serial
import ctypes
import struct
import numpy as np
from threading import Thread
import crc16
from openpyxl import Workbook
import os

class serial_node(serial.Serial):
    def __init__(self):
        super().__init__('COM6',115200)
        self.crc=crc16.crc16()

        self.s=struct.Struct('=B3f5f5fH')
        self.rx_buffer=ctypes.create_string_buffer(self.s.size)
        self.rx_data=np.zeros(1,dtype=self.rx_bag)

        self.wb=Workbook()
        self.ws=self.wb.active
        self.ws.title = "Sheet1"
        self.ws.append(['imu_r','imu_p','imu_y',
                        'light1','light2','light3','light4','light5',
                        'electric1','electric2','electric3','electric4','electric5'])

        self.excel_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data.xlsx")

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
                if self.inWaiting()==self.s.size:
                    self.rx_buffer=self.read(self.s.size)
                    if self.rx_buffer[0]==0x5A and self.crc.verify(self.rx_buffer,self.s.size):
                        self.unpacked_data = self.s.unpack_from(self.rx_buffer,0)
                        self.rx_data['header']=self.unpacked_data[0]
                        self.rx_data['imu_rpy'][0]=[
                            float(self.unpacked_data[1]),
                            float(self.unpacked_data[2]),
                            float(self.unpacked_data[3])]
                        self.rx_data['light'][0]=[
                            float(self.unpacked_data[4]),
                            float(self.unpacked_data[5]),
                            float(self.unpacked_data[6]),
                            float(self.unpacked_data[7]),
                            float(self.unpacked_data[8]),
                        ]
                        self.rx_data['electric'][0]=[
                            float(self.unpacked_data[9]),
                            float(self.unpacked_data[10]),
                            float(self.unpacked_data[11]),
                            float(self.unpacked_data[12]),
                            float(self.unpacked_data[13]),
                        ]
                        self.rx_data['crc_sum']=self.unpacked_data[14]
                        self.ws.append((list)(np.hstack((self.rx_data['imu_rpy'][0],self.rx_data['light'][0],self.rx_data['electric'][0]))))
                        self.wb.save(self.excel_path)
                        print(self.rx_data)

                else:
                    None
            except serial.SerialException:
                None
            # except:
            #     self.serial_restart()

def main():
    node=serial_node()
    node.serial_open()

    executor_thread = Thread(target=node.receive, daemon=True)
    executor_thread.start()
    executor_thread.join()

if __name__ == "__main__":
 	main()
