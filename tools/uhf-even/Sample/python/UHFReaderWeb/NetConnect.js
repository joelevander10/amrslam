var NetConnect = {
    setEnabled(bool) {
        if (!bool) {
            $('#port').attr('readonly', true);
            $('#cmbComBaud').attr('readonly', true)
            $('#idAddr').attr('readonly', true)
            $('#net_port').attr('readonly', true)
            $('#btnSportOpen').attr('disabled', 'disabled');
            $('#btnSportClose').attr('disabled', 'disabled');
            $('#btnConnect').attr('disabled', 'disabled');
        } else {
            $('#port').attr('readonly', false);
            $('#cmbComBaud').attr('readonly', false)
            $('#idAddr').attr('readonly', false)
            $('#net_port').attr('readonly', false)
            $('#btnSportOpen').removeAttr('disabled');
            $('#btnSportClose').removeAttr('disabled');
            $('#btnConnect').removeAttr('disabled');
        }
    },
    getVal() {
        var ip = $.trim($('#idAddr').val());
        var port = $.trim($('#net_port').val());
        return { ip: ip, port: port };
    },
    initEvent() {
        var self = this;
        $(document).on('click', '#btnConnect', function (e) {
            try {
                var opts = self.getVal();
                if (opts.ip.length == 0) {
                    Log.alertLog('Connect Failed，please input IP address');
                    return;
                }
                if (!/(?:(?:1[0-9][0-9]\.)|(?:2[0-4][0-9]\.)|(?:25[0-5]\.)|(?:[1-9][0-9]\.)|(?:[0-9]\.)){3}(?:(?:1[0-9][0-9])|(?:2[0-4][0-9])|(?:25[0-5])|(?:[1-9][0-9])|(?:[0-9]))/.test(opts.ip)) {
                    Log.alertLog('Error IPV4 Address');
                    return;
                }
                if (opts.port.length == 0) {
                    Log.alertLog('Please input port');
                    return;
                }
                if (opts.port == 0 || !/^([0-9]|[1-9]\d{1,3}|[1-5]\d{4}|6[0-4]\d{4}|65[0-4]\d{2}|655[0-2]\d|6553[0-5])$/.test(opts.port)) {
                    Log.alertLog('port num must 0~65535');
                    return;
                }
                if (Reader.IsOpened) {
                    Log.alertLog('reader is already opened ，please close first');
                    return;
                }
                Reader.OpenByIp(opts.ip, opts.port, 3000, function () {
                    Reader.InitReader();
                });
                Log.setInfo('reader open success，IP address：' + opts.ip + '，port' + opts.port);
                self.setEnabled(false)
            } catch (e) {
                if (Reader.IsOpened) {
                    Reader.Close()
                }
                Log.setError('open reader fail ' + e.message)
                self.setEnabled(true);
                Log.alertLog('open reader fail：' + e.message)
            }

        });
        $(document).on('click', '#btnDisConnect', function (e) {
            try {
                Log.setInfo('Reader Disconnected');
                self.setEnabled(true);
                if (Reader.m_handler == 0) {
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
                Log.alertLog(e.message);
            }
        });
    },
    init() {
        this.initEvent()
    }
};