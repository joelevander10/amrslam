var SerialConnect = {
    port: '',
    buad: 4,
    getPort() {
        var self = this;
        $.ajax({
            type: "POST",
            url: baseUrl + "/getPorts",
            dataType: "json",
            success: function (res) {
                var data = res.data
                if (res.code == 200 && data.res_code == 1000) {
                    self.initPortDom(data.ports);
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
    initPortDom(ports = []) {
        var portOpts = [];
        ports.forEach(function (item) {
            portOpts.push("<option value='" + item + "'>" + item + "</option>")
        })
        $('#port').html(portOpts.join())
        if (ports.length) {
            $('#port').val(ports[0])
            this.port = ports[0];
        }
    },
    setEnabled(bool) {
        if (!bool) {
            $('#port').attr('disabled', 'disabled');
            $('#btnSportOpen').attr('disabled', 'disabled');
        } else {
            $('#port').removeAttr('disabled');
            $('#btnSportOpen').removeAttr('disabled');
        }
    },

    bindEvent() {
        var self = this;
        $(document).on('change', '#port', function (e) {
            self.port = e.target.value;
        });
        $(document).on('change', '#cmbComBaud', function (e) {
            self.buad = e.target.value;
        });
        $(document).on('click', '#btnSportOpen', function (e) {
            try {
                var port = $.trim(self.port)
                if (port.length == 0) {
                    Log.alertLog('Failed to open the serial port, please enter the serial port number');
                    return;
                }
                if (Reader.IsOpene) {
                    Log.alertLog('The reader is already open, please close the reader first');
                    return;
                }
                Reader.OpenByPort(self.port, self.buad, function () {
                    Reader.InitReader();
                });
                Log.setInfo('The reader is opened successfully, the serial port number：' + self.port);
                self.setEnabled(false)
            } catch (e) {
                if (Reader.IsOpened) {
                    Reader.Close()
                }
                Log.setError('Reader failed to open ' + e.message)
                self.setEnabled(false);
                Log.alertLog('Reader failed to open：' + e.message)
            }

        });
        $(document).on('click', '#btnSportClose', function (e) {
            try {
                Log.setInfo('Close Reader');
                self.setEnabled(true);
                if (!Reader.m_handler) {
                    return;
                }
                $('#btninvStop').click();
                var timer = setTimeout(function () {
                    if (timer) {
                        clearTimeout(timer);
                        self.setEnabled(true);
                        Reader.Close();
                    }
                }, 1000)

            } catch (e) {
                Log.alertLog(e.message)
            }
        });
    },
    init() {
        $('#cmbComBaud').val(this.buad)
        this.getPort();
        this.bindEvent();
        if (Reader.IsOpened) {
            this.setEnabled(false);
        }
    }
};