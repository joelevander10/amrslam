from ctypes import *
from threading import Thread
import time
from uhf.error import UhfException

from uhf.handle import Api
from uhf.struct import TagInfo
from uhf.utils import hex_array_to_string

api = Api()



class InventoryThread(Thread):
    def __init__(self, hComm,  timeout): 
        super(InventoryThread, self).__init__()
        self.info = {}
        self.hComm = hComm

        self.timeout = timeout

        self.flag = False

    def run(self): 
        try:
            while True:
                try:
                    self.tagInfo = TagInfo()

                    if self.flag:
                        break

                    res = api.GetTagUii(self.hComm, self.tagInfo, 1000)
                    
                    lens = self.tagInfo.m_len
                    byte_array = list(self.tagInfo.m_code)
                    byte_array = byte_array[:lens]
                    code = hex_array_to_string(byte_array, lens)

                    if self.info.get(code):
                        if self.tagInfo.m_ant > 0 and self.tagInfo.m_ant <= 4:
                            self.info[code]["m_counts"][self.tagInfo.m_ant -1] += 1
                        self.info[code]["m_rssi"] = self.tagInfo.m_rssi / 10
                        self.info[code]["m_channel"] = self.tagInfo.m_channel
                    else:
                        m_counts = [0, 0, 0, 0]
                        m_counts[self.tagInfo.m_ant - 1] = 1

                        self.info[code] = {
                            'm_no': self.tagInfo.m_no,
                            'm_rssi': self.tagInfo.m_rssi / 10,          
                            "m_counts": m_counts,      
                            "m_channel": self.tagInfo.m_channel,        
                            "m_crc": list(self.tagInfo.m_crc),            
                            "m_pc": list(self.tagInfo.m_pc),             
                            "m_len": self.tagInfo.m_len,          
                            "m_code": code    
                        }

                except Exception as e:
                    if str(e) in ["-241", "-238"]:
                        continue
                    raise UhfException("The reader responds to a data format error or Waiting for reader response timed out")

                if res == -249:
                    break
                if self.tagInfo.m_ant == 0  or self.tagInfo.m_ant > 4:
                    continue
        except Exception as e:
            api.InventoryStop(self.hComm, 1000)
            
            
    def terminate(self):
        self.flag = True
