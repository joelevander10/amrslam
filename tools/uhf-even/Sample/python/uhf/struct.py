
from ctypes import *
class DeviceFullInfo(Structure):
    _fields_ = [
        ('DEVICEARRD', c_ubyte),
        ('RFIDPRO', c_ubyte),
        ('WORKMODE', c_ubyte),
        ('INTERFACE', c_ubyte),
        ('BAUDRATE', c_ubyte),
        ('WGSET', c_ubyte),
        ('ANT', c_ubyte),
        ('REGION', c_ubyte),
        ('STRATFREI', c_ubyte*2),
        ('STRATFRED', c_ubyte*2),
        ('STEPFRE', c_ubyte*2),
        ('CN', c_ubyte),
        ('RFIDPOWER', c_ubyte),
        ('INVENTORYAREA', c_ubyte),
        ('QVALUE', c_ubyte),
        ('SESSION', c_ubyte),
        ('ACSADDR', c_ubyte),
        ("ACSDATALEN", c_ubyte),
        ("FILTERTIME", c_ubyte),
        ("TRIGGLETIME", c_ubyte),
        ("BUZZERTIME", c_ubyte),
        ("INTERNELTIME", c_ubyte)
    ]

    
class DevicePara(Structure):
    _fields_ = [
        ('DEVICEARRD', c_ubyte),
        ('RFIDPRO', c_ubyte),
        ('WORKMODE', c_ubyte),
        ('INTERFACE', c_ubyte),
        ('BAUDRATE', c_ubyte),
        ('WGSET', c_ubyte),
        ('ANT', c_ubyte),
        ('REGION', c_ubyte),
        ('STRATFREI', c_ubyte*2),
        ('STRATFRED', c_ubyte*2),
        ('STEPFRE', c_ubyte*2),
        ('CN', c_ubyte),
        ('RFIDPOWER', c_ubyte),
        ('INVENTORYAREA', c_ubyte),
        ('QVALUE', c_ubyte),
        ('SESSION', c_ubyte),
        ('ACSADDR', c_ubyte),
        ("ACSDATALEN", c_ubyte),
        ("FILTERTIME", c_ubyte),
        ("TRIGGLETIME", c_ubyte),
        ("BUZZERTIME", c_ubyte),
        ("INTERNELTIME", c_ubyte)
    ]

class TagInfo(Structure):
    _fields_ = [
        ('m_no', c_ushort),
        ('m_rssi', c_short),
        ("m_ant", c_ubyte),
        ("m_channel", c_ubyte),
        ("m_crc", c_ubyte*2),
        ("m_pc", c_ubyte*2),
        ("m_len", c_ubyte),
        ("m_code", c_ubyte*255)
    ]