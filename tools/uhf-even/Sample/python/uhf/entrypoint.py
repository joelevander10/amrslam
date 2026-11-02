import time
import serial
import serial.tools.list_ports

from ctypes import *
from copy import deepcopy

from flask_restful import reqparse

from flask import Flask, current_app, render_template
from flask_cors import CORS

from uhf.conf import ERROR_CODE
from uhf.struct import DeviceFullInfo, DevicePara, TagInfo
from uhf.task import InventoryThread

from uhf.utils import res_jsonify
from uhf.handle import Api

api = Api()


class UHFServices:
    app = Flask(
        __name__, 
        static_url_path='', 
        static_folder="../UHFPrimeReaderWeb", 
        template_folder='../UHFPrimeReaderWeb')

    CORS(app)

    app.lib = cdll.LoadLibrary("./UHFPrimeReader.dll")   

    def serve(self, host="0.0.0.0", port="8888", threaded=True, debug=False):
        self.app.run(host=host, port=port, threaded=threaded, debug=debug)


S = UHFServices()


@UHFServices.app.route("/", methods=["get"])
def index():
    return render_template("index.html")


@UHFServices.app.route("/getPorts", methods=["post"])
def getPorts():
    comList = list(serial.tools.list_ports.comports())
    comAttr = [list(comList[i])[0] for i in range(len(comList))]
    return res_jsonify(0, ports = comAttr)


@UHFServices.app.route("/OpenDevice", methods=["post"])
def OpenDevice():
    """连接串口"""
    parser = reqparse.RequestParser()
    parser.add_argument('strComPort', type=str, default="0xFF")
    parser.add_argument('Baudrate', type=int, default=4)
    args = parser.parse_args()

    baud = {0: 9600, 1: 19200, 2:38400, 3: 57600, 4: 115200}
    
    ser = serial.Serial()
    ser.baudrate = baud[args["Baudrate"]] 
    ser.port = args["strComPort"]

    try:
        ser.open()
    except Exception as e:
        msgB="The reader is already open, please close the reader first"
        return res_jsonify(1001, msgB=msgB)
    else:
        ser.close()

    hComm = c_int()
    strComPort = c_char_p(args["strComPort"].encode('utf-8'))
    Baudrate = c_ubyte(args["Baudrate"])

    res = api.OpenDevice(hComm, strComPort, Baudrate)
    if res != 0:
        log, msgB = [f"serial {args['strComPort']} open fail"] * 2
        return res_jsonify(res, msgB= msgB, log=log, success=False)

    current_app.hComm = hComm.value
    log = f"The reader is opened successfully, the serial port number：{args['strComPort']}"
    return res_jsonify(res, log=log, hComm = hComm.value)


@UHFServices.app.route("/OpenNetConnection", methods=["post"])
def OpenNetConnection():
    """连接网口"""
    parser = reqparse.RequestParser()
    parser.add_argument('ip', type=str, default="192.168.1.200")
    parser.add_argument('port', type=int, required=True, help="输入读卡端口")
    parser.add_argument('timeoutMs', type=int, required=True)
    args = parser.parse_args()

    hComm = c_int()
    ip = c_char_p(args["ip"].encode('utf-8'))
    port = c_ushort(args["port"])
    timeoutMs = c_int(args["timeoutMs"])

    res = api.OpenNetConnection(hComm, ip, port, timeoutMs)
    if res != 0:
        msgB, log = [f"open reader fail IP: {ip.value} port: {port} connecting fail"] * 2
        return res_jsonify(res, msgB, log, False)
    
    current_app.hComm = hComm.value
    log = f"reader open success，IP address：{args['ip']}，port：{args['port']}"
    return res_jsonify(res, log=log, hComm = hComm.value)


@UHFServices.app.route("/CloseDevice", methods=["post"])
def CloseDevice():
    """关闭与读写器连接的串口或者网口。"""
    parser = reqparse.RequestParser()
    parser.add_argument('hComm', type=int, default=0)
    args = parser.parse_args()

    hComm = c_int(args["hComm"])

    res = api.CloseDevice(hComm)
    current_app.hComm = 0
    log = "Close Reader"
    return res_jsonify(res, log=log, success=True)


@UHFServices.app.route("/GetTagInfo", methods=["post"])
def GetTagInfo():
    """获取标签信息"""
    info = deepcopy(current_app.t.info)
    tag_info = list(info.values())

    for tag in tag_info:
        tag["m_counts"] = "/".join([str(x) for x in tag["m_counts"]])

    return res_jsonify(0, taginfo = tag_info)



@UHFServices.app.route("/SetRFPower", methods=["post"])
def SetRFPower():
    """设置功率命令"""
    parser = reqparse.RequestParser()
    parser.add_argument('hComm', type=int, default=0)
    parser.add_argument('power', type=int)
    parser.add_argument('reserved', type=int)
    args = parser.parse_args()

    hComm = c_int(args["hComm"])
    power = c_ubyte(args["power"])
    reserved = c_ubyte(args["reserved"])

    res = api.SetRFPower(hComm, power, reserved)
    return res_jsonify(res)


@UHFServices.app.route("/GetDevicePara", methods=["post"])
def GetDevicePara():
    """获取设备参数命令"""
    parser = reqparse.RequestParser()
    parser.add_argument('hComm', type=int, default=0)
    args = parser.parse_args()

    if current_app.hComm == 0:
        log, msgB = ["Failed to get Power, Reader is not open"] * 2
        return res_jsonify(1001, msgB, log, False)
    
    param = DeviceFullInfo()
    hComm = c_int(args["hComm"])

    res = api.GetDevicePara(hComm, param)

    if res != 0:
        log, msgB = [f"Failed to get Power, {ERROR_CODE.get(res)}"] * 2
        return res_jsonify(res, msgB, log, False)

    STRATFREI = list(param.STRATFREI)
    STRATFRED = list(param.STRATFRED)
    STEPFRE = list(param.STEPFRE)

    freq = STRATFREI[0]*256 + STRATFREI[1]
    freqde = STRATFRED[0]*256 + STRATFRED[1]
    step = STEPFRE[0]*256 + STEPFRE[1]

    freq = '{:.3f}'.format((freq * 1000 + freqde) / 1000)

    info = {
        "DEVICEARRD": param.DEVICEARRD, "RFIDPRO": param.RFIDPRO, "WORKMODE": param.WORKMODE, 
        "INTERFACE": param.INTERFACE, "BAUDRATE": param.BAUDRATE, "WGSET": param.WGSET, 
        "ANT": param.ANT, "REGION": param.REGION, "STRATFREI": freq, 
        "STRATFRED": freqde, "STEPFRE": step, "CN": param.CN, 
        "RFIDPOWER": param.RFIDPOWER, "INVENTORYAREA": param.INVENTORYAREA, "QVALUE": param.QVALUE, 
        "SESSION": param.SESSION, "ACSADDR": param.ACSADDR, "ACSDATALEN": param.ACSDATALEN, 
        "FILTERTIME": param.FILTERTIME, "TRIGGLETIME": param.TRIGGLETIME, "BUZZERTIME": param.BUZZERTIME, 
        "INTERNELTIME": param.INTERNELTIME
    }
    return res_jsonify(res, **info)


@UHFServices.app.route("/SetDevicePara", methods=["post"])
def SetDevicePara():
    """设置设备参数命令"""
    parser = reqparse.RequestParser()
    parser.add_argument('hComm', type=int, default=0)
    parser.add_argument('DEVICEARRD', type=int)
    parser.add_argument('RFIDPRO', type=int)
    parser.add_argument('WORKMODE', type=int)
    parser.add_argument('INTERFACE', type=int)
    parser.add_argument('BAUDRATE', type=int)
    parser.add_argument('WGSET', type=int)
    parser.add_argument('ANT', type=int)
    parser.add_argument('REGION', type=int)
    parser.add_argument('STRATFREI', type=str)
    parser.add_argument('STRATFRED', type=str)
    parser.add_argument('STEPFRE', type=int)
    parser.add_argument('CN', type=int)
    parser.add_argument('RFIDPOWER', type=int)
    parser.add_argument('INVENTORYAREA', type=int)
    parser.add_argument('QVALUE', type=int)
    parser.add_argument('SESSION', type=int)
    parser.add_argument('ACSADDR', type=int)
    parser.add_argument('ACSDATALEN', type=int)
    parser.add_argument('FILTERTIME', type=int)
    parser.add_argument('TRIGGLETIME', type=int)
    parser.add_argument('BUZZERTIME', type=int)
    parser.add_argument('INTERNELTIME', type=int)
    args = parser.parse_args()

    if current_app.hComm == 0:
        log, msgB = ["Failed to get Power, Reader is not open"] * 2
        return res_jsonify(1001, msgB, log, False)

    freq = int(float(args["STRATFREI"]))
    freqde = int(float(args["STRATFREI"]) * 1000 - freq * 1000)
    
    STRATFREI = (c_ubyte*2)(freq>>8, freq&0xff)
    STRATFRED = (c_ubyte*2)(freqde>>8, freqde&0xff)
    STEPFRE = (c_ubyte*2)(args["STEPFRE"]>>8, args["STEPFRE"]&0xff)

    param = DevicePara(
        args["DEVICEARRD"], args["RFIDPRO"], args["WORKMODE"], args["INTERFACE"], args["BAUDRATE"], 
        args["WGSET"], args["ANT"], args["REGION"], STRATFREI, STRATFRED, STEPFRE, args["CN"], args["RFIDPOWER"], 
        args["INVENTORYAREA"],  args["QVALUE"], args["SESSION"], args["ACSADDR"], args["ACSDATALEN"], 
        args["FILTERTIME"],  args["TRIGGLETIME"], args["BUZZERTIME"], args["INTERNELTIME"]
    )
    hComm = c_int(args["hComm"])

    try:
        res = api.SetDevicePara(hComm, param)
    except Exception as e:
        msgB, log = [str(e)] * 2
        return res_jsonify(1001, msgB, log, False)
    return res_jsonify(res)



@UHFServices.app.route("/StartCounting", methods=["post"])
def startCounting():
    """开始"""
    parser = reqparse.RequestParser()
    parser.add_argument('hComm', type=int, default=0)
    parser.add_argument('DEVICEARRD', type=int)
    parser.add_argument('RFIDPRO', type=int)
    parser.add_argument('WORKMODE', type=int)
    parser.add_argument('INTERFACE', type=int)
    parser.add_argument('BAUDRATE', type=int)
    parser.add_argument('WGSET', type=int)
    parser.add_argument('ANT', type=int)
    parser.add_argument('REGION', type=int)
    parser.add_argument('STRATFREI', type=str)
    parser.add_argument('STRATFRED', type=str)
    parser.add_argument('STEPFRE', type=int)
    parser.add_argument('CN', type=int)
    parser.add_argument('RFIDPOWER', type=int)
    parser.add_argument('INVENTORYAREA', type=int)
    parser.add_argument('QVALUE', type=int)
    parser.add_argument('SESSION', type=int)
    parser.add_argument('ACSADDR', type=int)
    parser.add_argument('ACSDATALEN', type=int)
    parser.add_argument('FILTERTIME', type=int)
    parser.add_argument('TRIGGLETIME', type=int)
    parser.add_argument('BUZZERTIME', type=int)
    parser.add_argument('INTERNELTIME', type=int)
    args = parser.parse_args()

    freq = int(float(args["STRATFREI"]))
    freqde = int(float(args["STRATFREI"]) * 1000 - freq * 1000)
    
    STRATFREI = (c_ubyte*2)(freq>>8, freq&0xff)
    STRATFRED = (c_ubyte*2)(freqde>>8, freqde&0xff)
    STEPFRE = (c_ubyte*2)(args["STEPFRE"]>>8, args["STEPFRE"]&0xff)

    param = DevicePara(
        args["DEVICEARRD"], args["RFIDPRO"], args["WORKMODE"], args["INTERFACE"], args["BAUDRATE"], 
        args["WGSET"], args["ANT"], args["REGION"], STRATFREI, STRATFRED, STEPFRE, args["CN"], args["RFIDPOWER"], 
        args["INVENTORYAREA"],  args["QVALUE"], args["SESSION"], args["ACSADDR"], args["ACSDATALEN"], 
        args["FILTERTIME"],  args["TRIGGLETIME"], args["BUZZERTIME"], args["INTERNELTIME"]
    )
    hComm = c_int(args["hComm"])
    
    try:
        if args["WORKMODE"] == 0:
            if current_app.inventory:
                current_app.t.terminate()
                api.InventoryStop(hComm, 10000)
            api.SetDevicePara(hComm, param)
            time.sleep(0.1)
            api.InventoryContinue(hComm, c_ubyte(0), c_int(0))
            current_app.work_mode = 0
        else:
            if current_app.inventory:
                current_app.t.terminate()
            api.SetDevicePara(hComm, param)
            time.sleep(0.1)
            current_app.work_mode = 1

        timeout = c_int(1000)

        current_app.inventory = True

        t = InventoryThread(hComm, timeout)  
        t.setDaemon(True)
        current_app.t = t
        current_app.t.start()
    except Exception as e:
        msgB, log = [f"Inventory label failed：{str(e)}"] * 2
        return res_jsonify(1001, msgB, log,  False)
    msgB = "Start inventory"
    return res_jsonify(0, msgB)


@UHFServices.app.route("/InventoryStop", methods=["post"])
def InventoryStop():
    """停止盘点"""
    parser = reqparse.RequestParser()
    parser.add_argument('hComm', type=int, default=0)
    parser.add_argument('timeout', type=int, required=True, help="等待数据时间，单位ms")
    args = parser.parse_args()

    hComm = c_int(args["hComm"])
    timeout = c_int(args["timeout"])
    if current_app.hComm == 0:
        return res_jsonify(0)

    try:
        res = 0
        if current_app.inventory:
            current_app.t.terminate()
            if current_app.work_mode == 0:
                res = api.InventoryStop(hComm, 1000)
            current_app.inventory = False
    except Exception as e:
        msgB = str(e)
        return res_jsonify(1000, msgB, "", False)

    log = "Inventory Stoped"
    return res_jsonify(res, "", log)


@UHFServices.app.route("/Close_Relay", methods=["post"])
def Close_Relay():
    """设置继电器闭合"""
    parser = reqparse.RequestParser()
    parser.add_argument('hComm', type=int)
    parser.add_argument('time', type=int, default=0)
    args = parser.parse_args()

    if current_app.hComm == 0:
        log, msgB = ["Failed to get Power, Reader is not open"] * 2
        return res_jsonify(1001, msgB, log, False)

    hComm = c_int(args["hComm"])
    time = c_ubyte(0)

    try:
        res = api.Close_Relay(hComm, time)
    except Exception as e:
        msgB = str(e)
        return res_jsonify(1001, msgB, "", False)
    log = "Set device Close_Relay Success"
    return res_jsonify(res, "", log)


@UHFServices.app.route("/Release_Relay", methods=["post"])
def Release_Relay():
    """设置继电器闭合"""
    parser = reqparse.RequestParser()
    parser.add_argument('hComm', type=int)
    args = parser.parse_args()

    if current_app.hComm == 0:
        log, msgB = ["Failed to get Power, Reader is not open"] * 2
        return res_jsonify(1001, msgB, log, False)

    hComm = c_int(args["hComm"])
    time = c_ubyte(0)
    
    try:
        res = api.Release_Relay(hComm, time)
    except Exception as e:
        msgB = str(e)
        return res_jsonify(1001, msgB, "", False)
    log = "Set device Release_Relay Success"
    return res_jsonify(res, "", log)

