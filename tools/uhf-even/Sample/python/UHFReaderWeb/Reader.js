var Reader = {
    m_handler: 0,
    m_iState: 0,
    m_ip: '',
    m_netPort: 0,
    m_sport: 0,
    m_timer: null,
    get m_handler() {
        return sessionStorage.getItem('m_handler') || 0;
    },
    set m_handler(value) {
        sessionStorage.setItem('m_handler', value)
    },
    get m_iState() {
        return sessionStorage.getItem('m_iState') || 0;
    },
    set m_iState(value) {
        sessionStorage.setItem('m_iState', value)
    },
    get m_ip() {
        return sessionStorage.getItem('m_ip') || '';
    },
    set m_ip(value) {
        sessionStorage.setItem('m_ip', value)
    },
    get m_netPort() {
        return sessionStorage.getItem('m_netPort') || 0;
    },
    set m_netPort(value) {
        sessionStorage.setItem('m_netPort', value)
    },
    get m_sport() {
        return sessionStorage.getItem('m_sport') || 0;
    },
    set m_sport(value) {
        sessionStorage.setItem('m_sport', value)
    },
    get IsOpened() {
        return this.m_iState != 0;
    },
    get timer() {
        return this.m_timer;
    },
    set timer(value) {
        this.m_timer = value;
    },
    InitReader() {
        if (this.IsOpened) {
            $.ajax({
                type: "POST",
                url: baseUrl + "/GetDevicePara",
                dataType: "json",
                contentType: 'application/json',
                data: JSON.stringify({ hComm: this.m_handler }),
                success: function (res) {
                    var data = res.data
                    if (res.code == 200 && data.res_code === 1000) {
                        devicepara.Addr = data.ACSADDR;
                        devicepara.Protocol = data.RFIDPRO;
                        devicepara.Baud = data.BAUDRATE;
                        devicepara.Workmode = data.WORKMODE;
                        devicepara.port = data.INTERFACE;
                        devicepara.wieggand = data.WGSET;
                        devicepara.Ant = data.ANT;
                        devicepara.Region = data.REGION;
                        devicepara.Channel = data.CN;
                        devicepara.Power = data.RFIDPOWER;
                        devicepara.Area = data.INVENTORYAREA;
                        devicepara.Q = data.QVALUE;
                        devicepara.Session = data.SESSION;
                        devicepara.Startaddr = data.ACSADDR;
                        devicepara.DataLen = data.ACSDATALEN;
                        devicepara.Filtertime = data.FILTERTIME;
                        devicepara.Triggletime = data.TRIGGLETIME;
                        devicepara.Buzzertime = data.BUZZERTIME;
                        devicepara.IntenelTime = data.INTERNELTIME;
                        devicepara.StartFreq = data.STRATFREI;
                        devicepara.Stepfreq = data.STEPFRE;
                        $('#cmbTxPower').val(devicepara.Power);
                        if (devicepara.Workmode > 1) {
                            $('#cmbWorkmode').val(0)
                        }
                        else {
                            $('#cmbWorkmode').val(devicepara.Workmode)
                        }
                        $('#cmbRegion').val(devicepara.Region);
                        ChannelRegion.initHZ(devicepara.Region);
                        $('#cmbFreqStart').val(devicepara.StartFreq);
                        var cmbFreqEnd = ((devicepara.StartFreq * 1000 + devicepara.Stepfreq * (devicepara.Channel - 1)) / 1000).toFixed(3)
                        $('#cmbFreqEnd').val(cmbFreqEnd);
                    }
                    if (data.log) {
                        Log.setLog(data.log)
                    }
                    if (data.msgB) {
                        Log.alertLog(data.msgB)
                    }
                }
            });
        }
    },
    OpenByPort(port, baudrate, callback) {
        var self = this;
        if (port == null) {
            Log.setError('port(Serial)');
            return;
        }
        port = $.trim(port)
        if (port.length == 0) {
            Log.setError('port(serial)：please input serial');
            return;
        }
        if (this.m_iState != 0) {
            Log.setError('reader is already opened');
            return;
        }

        $.ajax({
            type: "POST",
            url: baseUrl + "/OpenDevice",
            contentType: 'application/json',
            dataType: "json",
            data: JSON.stringify({ strComPort: port, Baudrate: baudrate }),
            success: function (res) {
                var data = res.data
                if (res.code == 200 && data.res_code === 1000) {
                    self.m_sport = port;
                    self.m_iState = 1;
                    self.m_handler = data.hComm;
                    callback();
                }

                if (data.log) {
                    Log.setLog(data.log)
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB)
                }
            }
        });
    },
    OpenByIp(ip, port, timeoutMs, callback) {
        var self = this;
        if (ip == null) {
            Log.setError('ip(addr)');
            return;
        }
        ip = $.trim(ip)
        if (ip.length == 0) {
            Log.setError('ip(addr)：please input IP addr');
            return;
        }
        if (port == 0) {
            Log.setError('reader is already opened');
            return;
        }
        if (this.m_iState != 0) {
            Log.setError('port(port)：port can not be 0');
            return;
        }
        $.ajax({
            type: "POST",
            url: baseUrl + "/OpenNetConnection",
            dataType: "json",
            contentType: 'application/json',
            data: JSON.stringify({ ip: ip, port: port, timeoutMs: timeoutMs }),
            success: function (res) {
                var data = res.data
                if (res.code == 200 && data.res_code === 1000) {
                    self.m_sport = port;
                    self.m_iState = 1;
                    self.m_handler = data.hComm;
                    callback();
                }

                if (data.log) {
                    Log.setLog(data.log)
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB)
                }
            }
        });
    },
    Close() {
        var self = this;
        if (this.m_handler != 0) {
            try {
                $.ajax({
                    type: "POST",
                    url: baseUrl + "/CloseDevice",
                    contentType: 'application/json',
                    dataType: "json",
                    data: JSON.stringify({ hComm: self.m_handler }),
                    success: function (res) {
                        var data = res.data
                        if (data.log) {
                            Log.setLog(data.log)
                        }
                        if (data.msgB) {
                            Log.alertLog(data.msgB)
                        }
                    }
                });
            } catch { }
            self.m_handler = 0;
        }
        this.m_iState = 0;
    },
    Close_Relay(time) {
        if (this.m_iState == 0) {
            Log.alertLog('Reader is not open');
            return;
        }
        $.ajax({
            type: "POST",
            url: baseUrl + "/Close_Relay",
            dataType: "json",
            contentType: 'application/json',
            data: JSON.stringify({ hComm: this.m_handler, time: time }),
            success: function (res) {
                var data = res.data
                if (data.log) {
                    Log.setLog(data.log)
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB)
                }
            }
        });
    },
    Release_Realy(time) {
        if (this.m_iState == 0) {
            Log.alertLog('Reader is not open');
            return;
        }
        $.ajax({
            type: "POST",
            url: baseUrl + "/Release_Relay",
            dataType: "json",
            contentType: 'application/json',
            data: JSON.stringify({ hComm: this.m_handler, time: time }),
            success: function (res) {
                var data = res.data
                if (data.log) {
                    Log.setLog(data.log)
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB)
                }
            }
        });
    },
    SetRfTxPower(txPower, reserved) {
        if (this.m_iState == 0) {
            Log.alertLog('Reader is not open');
            return;
        }
        $.ajax({
            type: "POST",
            url: baseUrl + "/SetRFPower",
            dataType: "json",
            contentType: 'application/json',
            data: JSON.stringify({ hComm: self.m_handler, txPower: txPower, reserved: reserved }),
            success: function (res) {
                var data = res.data
                if (data.log) {
                    Log.setLog(data.log);
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB);
                }
            }
        });
    },
    GetDevicePara() {
        if (this.m_iState == 0) {
            Log.setError('Reader is not open');
            return;
        }
        this.InitReader();
    },
    GetTagInfo() {
        if (this.m_iState == 0) {
            Log.setError('Reader is not open');
            return;
        }

        $.ajax({
            type: "POST",
            url: baseUrl + "/GetTagInfo",
            dataType: "json",
            success: function (res) {
                var data = res.data;
                if (data.log) {
                    Log.setLog(data.log);
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB);
                }
                if (res.code == 200 && data.res_code == 1000) {
                    var taginfo = data.taginfo || [];
                    var tbody = [];
                    taginfo.forEach((item, index) => {
                        tbody.push('<tr>');
                        tbody.push('<td>' + (index + 1) + '</td>')
                        tbody.push('<td>' + item.m_code + '</td>')
                        tbody.push('<td>' + item.m_len + '</td>')
                        tbody.push('<td>' + item.m_counts + '</td>')
                        tbody.push('<td>' + item.m_rssi + '</td>')
                        tbody.push('<td>' + item.m_channel + '</td>')
                        tbody.push('<tr>');
                    })

                    $('#data_table_body').html(tbody.join());
                }
            }
        });

    },
    SetDevicePara(devicepara) {
        if (this.m_iState == 0) {
            Log.setError('Reader is not open');
            return;
        }

        devicepara.Region = $('#cmbRegion').val();

        var starfreq = $('#cmbFreqStart').val();
        var endfreq = $('#cmbFreqEnd').val();
        if (starfreq == '') {
            Log.setError('Please select a starting frequency');
            return;
        }
        if (endfreq == '') {
            Log.setError('Please select a ending frequency');
            return;
        }
        var cmbFreqStartIndex = 0, cmbFreqEndIndex = 0;
        $('#cmbFreqStart option').each(function (ele, index) {
            if ($(index).html() == starfreq) {
                cmbFreqStartIndex = ele;
            }
        })
        $('#cmbFreqEnd option').each(function (ele, index) {
            if ($(index).html() == endfreq) {
                cmbFreqEndIndex = ele;
            }
        })
        var count = cmbFreqEndIndex - cmbFreqStartIndex + 1;

        var cmbRegion = $('#cmbRegion').find('option:selected');
        devicepara.StartFreq = Number(starfreq).toFixed(3);
        devicepara.StartFreqde = Number(endfreq).toFixed(3);
        devicepara.Stepfreq = cmbRegion.data('step')
        devicepara.Channel = count;


        $.ajax({
            type: "POST",
            url: baseUrl + "/SetDevicePara",
            dataType: "json",
            contentType: 'application/json',
            data: JSON.stringify({
                hComm: this.m_handler, ACSADDR: devicepara.ACSADDR,
                ACSDATALEN: devicepara.ACSDATALEN,
                ANT: devicepara.ANT,
                BAUDRATE: devicepara.BAUDRATE,
                BUZZERTIME: devicepara.BUZZERTIME,
                CN: devicepara.CN,
                DEVICEARRD: devicepara.DEVICEARRD,
                FILTERTIME: devicepara.FILTERTIME,
                INTERNELTIME: devicepara.INTERNELTIME,
                INTERFACE: devicepara.INTERFACE,
                INVENTORYAREA: devicepara.INVENTORYAREA,
                QVALUE: devicepara.QVALUE,
                REGION: devicepara.REGION,
                RFIDPOWER: devicepara.RFIDPOWER,
                RFIDPRO: devicepara.RFIDPRO,
                SESSION: devicepara.SESSION,
                STEPFRE: (devicepara.STEPFRE),
                STRATFRED: (devicepara.STRATFRED),
                STRATFREI: (devicepara.STRATFREI),
                TRIGGLETIME: devicepara.TRIGGLETIME,
                WGSET: devicepara.WGSET,
                WORKMODE: devicepara.WORKMODE
            }),
            success: function (res) {
                var data = res.data;
                if (data.log) {
                    Log.setLog(data.log);
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB);
                }
            }
        })
    },
    StartCounting(callback) {
        if (this.m_iState == 0) {
            Log.setError('Reader is not open');
            return;
        }
        $.ajax({
            type: "POST",
            url: baseUrl + "/StartCounting",
            dataType: "json",
            contentType: 'application/json',
            data: JSON.stringify({
                hComm: this.m_handler,
                ACSADDR: devicepara.ACSADDR,
                ACSDATALEN: devicepara.ACSDATALEN,
                ANT: devicepara.ANT,
                BAUDRATE: devicepara.BAUDRATE,
                BUZZERTIME: devicepara.BUZZERTIME,
                CN: devicepara.CN,
                DEVICEARRD: devicepara.DEVICEARRD,
                FILTERTIME: devicepara.FILTERTIME,
                INTERNELTIME: devicepara.INTERNELTIME,
                INTERFACE: devicepara.INTERFACE,
                INVENTORYAREA: devicepara.INVENTORYAREA,
                QVALUE: devicepara.QVALUE,
                REGION: devicepara.REGION,
                RFIDPOWER: devicepara.RFIDPOWER,
                RFIDPRO: devicepara.RFIDPRO,
                SESSION: devicepara.SESSION,
                STEPFRE: (devicepara.STEPFRE),
                STRATFRED: (devicepara.STRATFRED),
                STRATFREI: (devicepara.STRATFREI),
                TRIGGLETIME: devicepara.TRIGGLETIME,
                WGSET: devicepara.WGSET,
                WORKMODE: devicepara.WORKMODE
            }),
            success: function (res) {
                var data = res.data;
                if (res.code == 200 && data.res_code == 1000) {
                    callback()
                }
                if (data.log) {
                    Log.setLog(data.log)
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB)
                }
            }
        })
    },
    InventoryStop(timeout) {
        if (this.m_iState == 0) {
            Log.setError('Reader is not open');
            return;
        }

        $.ajax({
            type: "POST",
            url: baseUrl + "/InventoryStop",
            dataType: "json",
            contentType: 'application/json',
            data: JSON.stringify({
                hComm: this.m_handler,
                timeout: timeout
            }),
            success: function (res) {
                var data = res.data;

                if (data.log) {
                    Log.setLog(data.log)
                }
                if (data.msgB) {
                    Log.alertLog(data.msgB)
                }
            }
        })
    },
    // Clear() {
    //     if (this.m_iState == 0) {
    //         Log.setError('Reader is not open');
    //         return;
    //     }
    //     $.ajax({
    //         type: "POST",
    //         url: baseUrl + "/clear",
    //         dataType: "json",
    //         contentType: 'application/json',
    //         success: function (res) {
    //             var data = res.data;
    //             $('#data_table_body').html('');
    //             if (data.log) {
    //                 Log.setLog(data.log)
    //             }
    //             if (data.msgB) {
    //                 Log.alertLog(data.msgB)
    //             }
    //         }
    //     })
    // },
    initEvent() {
        var self = this;
        $(document).on('click', '#btnGetTxPower', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                Log.setInfo('Get device RfPower');
                self.GetDevicePara();
                $('#cmbTxPower').val(devicepara.Power);
                Log.setInfo('Get device RfPower Success');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#btnSetTxPower', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                Log.setInfo('Set device RfPower');
                devicepara.Power = $('#cmbTxPower').val();
                Log.setInfo('Set device RfPower Success');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#btnGetWorkMode', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                Log.setInfo('Get device Workmode');
                self.GetDevicePara();
                $('#cmbWorkmode').val(devicepara.Workmode);
                Log.setInfo('Get device Workmode Success');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('change', '#cmbWorkmode', function (e) {
            Log.setInfo('Set device Workmode');
            devicepara.Workmode = e.target.value;
            Log.setInfo('Set device Workmode Success');
        })
        $(document).on('change', '#cmbTxPower', function (e) {
            Log.setInfo('Set device Power');
            devicepara.Power = e.target.value;
            Log.setInfo('Set device Power Success');
        })
        $(document).on('click', '#btnSetWorkMode', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                Log.setInfo('Set device Workmode');
                devicepara.Workmode = $('#cmbWorkmode').val();
                Log.setInfo('Set device Workmode Success');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#btnGetFreq', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                Log.setInfo('Get device Freq');
                self.GetDevicePara();
                var freq = new FreqInfo();
                freq.Region = devicepara.Region;
                freq.StartFreq = devicepara.StartFreq + devicepara.StartFreqde / 1000.0;
                freq.StepFreq = devicepara.Stepfreq;
                freq.Count = devicepara.Channel;
                $('#cmbRegion').val(freq.Region);
                $('#cmbFreqStart').val(Number(freq.StartFreq).toFixed(3))
                $('#cmbFreqEnd').val(Number(freq.StartFreq + freq.StepFreq * freq.Count).toFixed(3))
                Log.setInfo('Get device Freq Success');

            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#btnSetFreq', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                Log.setInfo('Set device Freq');
                var region = $('#cmbRegion').val();
                devicepara.Region = region;
                var starfreq;
                var endfreq;
                var count;
                //custom
                if (region == 0) {
                    starfreq = $.trim($('#cmbFreqStart_input').val());
                    if (starfreq.length == 0) {
                        Log.setError('Please enter the starting frequency');
                        return;
                    }
                    var step = 500;
                    endfreq = $.trim($('#cmbFreqEnd_input').val());
                    if (!isNaN(starfreq) || starfreq < 840 || starfreq > 960) {
                        Log.setError('The entered "frequency" value must be a number between 840 and 960 (including 840 and 960) (can be a decimal)');
                        return;
                    }
                    if (!isNaN(endfreq) || endfreq < 840 || endfreq > 960) {
                        Log.setError('The entered "frequency" value must be a number between 840 and 960 (including 840 and 960) (can be a decimal)');
                        return;
                    }
                    if (step < 0 || step > 2000) {
                        Log.setError('The entered "Channel Spacing Frequency" value must be an integer between 0 and 500 (Include an integer between 1 and 500)');
                        return;
                    }
                    count = (endfreq - starfreq) / step;
                    if (count < 1 || count > 50) {
                        Log.setError('The entered "Channel Spacing Frequency" value must be an integer between 1 and 50 (Include an integer between 1 and 50)')
                        return;
                    }
                } else {
                    var starfreq = $('#cmbFreqStart').val();
                    var endfreq = $('#cmbFreqEnd').val();
                    if (starfreq == '') {
                        Log.setError('Please select a starting frequency');
                        return;
                    }
                    if (endfreq == '') {
                        Log.setError('Please select a ending frequency');
                        return;
                    }
                    var cmbFreqStartIndex = $('#cmbFreqStart').attr('selectedIndex');
                    var cmbFreqEndIndex = $('#cmbFreqEnd').attr('selectedIndex');
                    count = cmbFreqEndIndex - cmbFreqStartIndex + 1;
                }
                var cmbRegion = $('#cmbRegion').find('option:selected');
                devicepara.StartFreq = Number(starfreq).toFixed(3);
                devicepara.StartFreqde = Number(endfreq).toFixed(3);
                devicepara.Stepfreq = cmbRegion.data('step')
                devicepara.Channel = count;

                self.SetDevicePara(devicepara);
                Log.setInfo('Set device Freq Success');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#Close_Realy', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                Log.setInfo('Set device Close_Realy');
                var time = 0;
                self.Close_Relay(time);
                Log.setInfo('Set device Close_Realy Success');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#Release_Realy', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                Log.setInfo('Set device Release_Realy');
                var time = 0;
                self.Release_Realy(time);
                Log.setInfo('Set device Release_Realy Success');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#btninvStop', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                $('#btnInventoryActive').removeAttr('disabled');
                var timeout = 10000;
                if (self.timer) {
                    clearInterval(self.timer);
                    // clearTimeout(self.timer)
                }
                self.InventoryStop(timeout);
                Log.setInfo('Inventory Stoped');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#btnInventoryActive', function () {
            try {
                if (self.m_handler == 0) {
                    Log.setError('Reader not connected');
                    return;
                }
                var interval = 40;
                self.StartCounting(function () {
                    $('#btnInventoryActive').attr('disabled', 'disabled');
                    self.timer = setInterval(() => {
                        self.GetTagInfo()
                    }, interval);
                    // self.timer = setTimeout(() => {
                    //     self.GetTagInfo()
                    // }, interval);
                });
                Log.setInfo('Start  inventory');
            } catch (e) {
                Log.alertLog(e.message)
            }
        })
        $(document).on('click', '#btnClear', function () {
            $('#data_table_body').html('');
        })
    },
    init() {
        this.initEvent()
        if (this.IsOpened) {
            this.InitReader();
        }
    }

};