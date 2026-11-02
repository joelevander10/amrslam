class FreqInfo {
    get Region() { return this.m_region }
    set Region(value) { this.m_region = value }

    get StartFreq() { return this.m_startFreq1 + this.m_startFreq2 / 1000.0 }
    set StartFreq(value) {
        this.m_startFreq1 = value;
        this.m_startFreq2 = (value - this.m_startFreq1) * 1000;
    }

    get StepFreq() { return this.m_stepFreq }
    set StepFreq(value) { this.m_stepFreq = value }

    get Count() { return this.m_cnt }
    set Count(value) { this.m_cnt = value }
}