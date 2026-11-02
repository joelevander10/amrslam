var ChannelRegion = {
    FreqStart: '',
    FreqEnd: '',
    channelRegion: [
        { hz: 0, step: 0, count: 0, index: 0, label: 'Custom' },
        { hz: 902.750, step: 500, count: 50, index: 1, label: 'USA' },
        { hz: 917.100, step: 200, count: 32, index: 2, label: 'Korea' },
        { hz: 865.100, step: 200, count: 15, index: 3, label: 'Europe' },
        { hz: 952.200, step: 200, count: 8, index: 4, label: 'Japan' },
        { hz: 919.500, step: 500, count: 7, index: 5, label: 'Malaysia' },
        { hz: 865.700, step: 600, count: 4, index: 6, label: 'Europe3' },
        { hz: 840.125, step: 250, count: 20, index: 7, label: 'China_1' },
        { hz: 920.125, step: 250, count: 20, index: 8, label: 'China_2' },
    ],
    initChannelRegionDom(selected) {
        var channelRegion = [];
        this.channelRegion.forEach(ele => {
            channelRegion.push("<option value='" + ele.index + "' data-step='" + ele.step + "' data-count='" + ele.count + "'>" + ele.label + "</option>")
        });
        $('#cmbRegion').html(channelRegion.join(''))
        $('#cmbRegion').val(selected)
    },
    getOptions(index) {
        var opts = this.channelRegion[index];
        var options = [];
        if (opts.index) {
            for (var i = 0; i < opts.count; i++) {
                var hz = (opts.hz + (opts.step / 1000) * i).toFixed(3);
                options.push(hz)
            }
        }
        return options;
    },
    initChannelRegionHz(opts) {
        var options = [];
        for (var i = 0; i < opts.length; i++) {
            options.push("<option value='" + opts[i] + "'>" + opts[i] + "</option>")
        }
        return options;
    },
    CalcChannel() {
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
            count = cmbFreqEndIndex - cmbFreqStartIndex + 1;
        }
        var cmbRegion = $('#cmbRegion').find('option:selected');
        devicepara.StartFreq = Number(starfreq).toFixed(3);
        devicepara.StartFreqde = Number(endfreq).toFixed(3);
        devicepara.Stepfreq = cmbRegion.data('step')
        devicepara.Channel = count;
    },
    BindEvent() {
        var self = this;
        $(document).on('change', '#cmbRegion', function (e) {
            var selected = e.target.value;
            self.initHZ(selected);
            self.CalcChannel();
        })
        $(document).on('change', '#cmbFreqStart', function (e) {
            self.FreqStart = e.target.value;
            self.CalcChannel();
        })
        $(document).on('change', '#cmbFreqEnd', function (e) {
            self.FreqEnd = e.target.value;
            self.CalcChannel();

        })
        $(document).on('change', '#cmbFreqStart_input', function (e) {
            self.FreqStart = e.target.value;
        })
        $(document).on('change', '#cmbFreqEnd_input', function (e) {
            self.FreqEnd = e.target.value;
        })
    },
    initHZ(index) {
        if (index == 0) {
            $('#cmbFreqStart').hide();
            $('#cmbFreqEnd').hide();

            $('#cmbFreqStart_input').show();
            $('#cmbFreqEnd_input').show();
        } else {
            var opts = this.getOptions(index);
            var options = this.initChannelRegionHz(opts);
            $('#cmbFreqStart').html(options);
            $('#cmbFreqEnd').html(options);

            this.FreqStart = opts[0]
            this.FreqEnd = opts[opts.length - 1]
            $('#cmbFreqStart').val(this.FreqStart);
            $('#cmbFreqEnd').val(this.FreqEnd);

            $('#cmbFreqStart').show();
            $('#cmbFreqEnd').show();

            $('#cmbFreqStart_input').hide();
            $('#cmbFreqEnd_input').hide();
        }

    },
    init(index = 0) {
        this.initChannelRegionDom(index);
        this.initHZ(index);
        this.BindEvent();
    }
}