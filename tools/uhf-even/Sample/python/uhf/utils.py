import time

from flask import jsonify

from uhf.conf import ERROR_CODE

def res_jsonify(res, msgB="", log="", success=True, **kwargs):
    if ERROR_CODE.get(res):
        data = ERROR_CODE.get(res)
    else:
        data = ""

    code = 1000 if res == 0 else 1001

    now_time = time.strftime('%Y/%m/%d %H:%M:%S', time.localtime())
    if log:
        ms = "info" if success else "error"
        log = f"{now_time} {ms}: {log}" if log else ""        

    res = {
        "res_code": code,
        "data": data,
        "msgB": msgB,
        "log": log
    }
    if kwargs:
        res.update(kwargs)
        
    return jsonify({
        "msg": "",
        "code": 200,
        "data": res
    })


def hex_array_to_string(array, len):
    if len == 0:
        return ""
    if len == 1:
        return hex(array[0]).replace("0x", "").zfill(2)
    string = []
    
    for i in range(len-1):
        string.append(hex(array[i]).replace("0x", "").zfill(2))
        string.append(" ")
    string.append(hex(array[-1]).replace("0x", "").zfill(2))

    return " ".join(string)

