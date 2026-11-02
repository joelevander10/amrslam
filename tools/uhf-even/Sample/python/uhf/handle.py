from ctypes import *
from uhf.conf import ERROR_CODE

from uhf.error import UhfException

class Api:
    def __init__(self):
        self.lib = cdll.LoadLibrary("./UHFPrimeReader.dll")  
        self.lib.OpenDevice.restype = c_int32
        self.lib.OpenNetConnection.restype = c_int32
        self.lib.CloseDevice.restype = c_int32
        self.lib.GetDevicePara.restype = c_int32
        self.lib.SetDevicePara.restype = c_int32
        self.lib.SetRFPower.restype = c_int32
        self.lib.Close_Relay.restype = c_int32
        self.lib.Release_Relay.restype = c_int32
        self.lib.GetTagUii.restype = c_int32
        self.lib.InventoryContinue.restype = c_int32
        self.lib.InventoryStop.restype = c_int32





    def OpenDevice(self, hComm, port, Baudrate):
        res = self.lib.OpenDevice(byref(hComm), port, Baudrate)
        return res

    def OpenNetConnection(self, hCom, ip, port, timeoutMs):
        res = self.lib.OpenNetConnection(byref(hCom), ip, port, timeoutMs)
        return res

    def CloseDevice(self, hComm):
        res = self.lib.CloseDevice(hComm)
        return res
    
    def GetDevicePara(self, hComm, param):
        res = self.lib.GetDevicePara(hComm, byref(param))
        return res

    def SetDevicePara(self, hComm, param):
        res = self.lib.SetDevicePara(hComm, param)
        if res != 0:
            raise UhfException(ERROR_CODE[res])
        return res

    def SetRFPower(self, hComm, power, reserved):
        res = self.lib.SetRFPower(hComm, power, reserved)
        return res

    def Close_Relay(self, hComm, time):
        res = self.lib.Close_Relay(hComm, time)
        if res != 0:
            raise UhfException(ERROR_CODE[res])
        return res

    def Release_Relay(self, hComm, time):
        res = self.lib.Release_Relay(hComm, time)
        if res != 0:
            raise UhfException(ERROR_CODE[res])
        return res

    def GetTagUii(self, hComm, tagInfo, timeout):
        res = self.lib.GetTagUii(hComm, byref(tagInfo), timeout)
        if res != 0:
            if res in [-241, -238]:
                raise UhfException(str(res))
            raise UhfException(ERROR_CODE[res])
        return res

    def InventoryContinue(self, hComm, invCount, invParam):
        res = self.lib.InventoryContinue(hComm, invCount, invParam)
        if res != 0:
            raise UhfException(ERROR_CODE[res])
        return res

    def InventoryStop(self, hComm, timeout):
        res = self.lib.InventoryStop(hComm, timeout)
        return 0