using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace WindowsFormsApp1
{
    /// <summary>
    /// 频率信息
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct FreqInfo
    {
        /// <summary>
        /// 地域索引
        /// </summary>
        byte m_region;
        /// <summary>
        /// 起始频率的整数部分，单位：MHz，取值范围为：840~960
        /// </summary>
        ushort m_startFreq1;
        /// <summary>
        /// 起始频率的小数部分，单位：MHz，取值范围为：0~999
        /// </summary>
        ushort m_startFreq2;
        /// <summary>
        /// 频率步进,单位：KHz，取值范围为：0~500
        /// </summary>
        ushort m_stepFreq;
        /// <summary>
        /// 信道数，取值范围为：1~50（包括1和50）
        /// </summary>
        byte m_cnt;

        /// <summary>
        /// 地域索引，取值范围：
        /// 0：China-1,
        /// 1：China-2,
        /// 2：FCC,
        /// 3：Japan,
        /// 4：Malaysia,
        /// 5：ETSI
        /// </summary>
        public byte Region
        {
            get { return m_region; }
            set { m_region = value; }
        }

        /// <summary>
        /// 起始频率，单位：MHz，取值范围为：840~960
        /// </summary>
        public float StartFreq
        {
            get { return m_startFreq1 + m_startFreq2 / 1000.0f; }
            set
            {
                m_startFreq1 = (ushort)(int)value;
                m_startFreq2 = (ushort)(int)((value - m_startFreq1) * 1000);
            }
        }

        /// <summary>
        /// 频率步进,单位：KHz，取值范围为：0~500
        /// </summary>
        public ushort StepFreq
        {
            get { return m_stepFreq; }
            set { m_stepFreq = value; }
        }

        /// <summary>
        /// 信道数，取值范围为：1~50（包括1和50）
        /// </summary>
        public byte Count
        {
            get { return m_cnt; }
            set { m_cnt = value; }
        }

    }
}
