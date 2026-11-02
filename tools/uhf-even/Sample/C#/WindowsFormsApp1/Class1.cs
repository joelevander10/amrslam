using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace WindowsFormsApp1
{

        [StructLayout(LayoutKind.Sequential)]
        public struct TagInfo
        {
            /// <summary>
            /// 标签序号
            /// </summary>
            private ushort m_no;
            /// <summary>
            /// RSSI，单位：0.1dBm
            /// </summary>
            private short m_rssi;
            /// <summary>
            /// 天线索引
            /// </summary>
            private byte m_ant;
            /// <summary>
            /// 信道
            /// </summary>
            private byte m_channel;
            /// <summary>
            /// CRC
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            private byte[] m_crc;
            /// <summary>
            /// 标签的PC或编码长度+编码头数据
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            private byte[] m_pc;
            /// <summary>
            /// code中有效数据的长度
            /// </summary>
            private byte m_len;
            /// <summary>
            /// 标签的响应数据，长度255个byte
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 255)]
            private byte[] m_code;

            /// <summary>
            /// 标签序号
            /// </summary>
            public ushort NO
            {
                get { return m_no; }
            }

            /// <summary>
            /// 标签的PC或编码长度+编码头数据，长度2个byte
            /// </summary>
            public byte[] PC
            {
                get { return m_pc; }
            }

            /// <summary>
            /// code中有效数据的长度
            /// </summary>
            public byte CodeLength
            {
                get { return m_len; }
            }

            /// <summary>
            /// 标签的响应数据，长度255个byte
            /// </summary>
            public byte[] Code
            {
                get { return m_code; }
            }

            /// <summary>
            /// RSSI，单位：0.1dBm
            /// </summary>
            public short Rssi
            {
                get { return m_rssi; }
            }

            /// <summary>
            /// 天线接口序号
            /// </summary>
            public byte Antenna
            {
                get { return m_ant; }
            }

            /// <summary>
            /// 信道
            /// </summary>
            public byte Channel
            {
                get { return m_channel; }
            }

            /// <summary>
            /// CRC
            /// </summary>
            public byte[] CRC
            {
                get { return m_crc; }
            }

        }


        /// <summary>
        /// 设备参数
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct Devicepara
        {

            private byte DEVICEARRD;
            private byte RFIDPRO;
            private byte WORKMODE;
            private byte INTERFACE;
            private byte BAUDRATE;
            private byte WGSET;
            private byte ANT;
            private byte REGION;
            private ushort STRATFREI;
            private ushort STRATFRED;
            private ushort STEPFRE;
            private byte CN;
            private byte RFIDPOWER;
            private byte INVENTORYAREA;
            private byte QVALUE;
            private byte SESSION;
            private byte ACSADDR;
            private byte ACSDATALEN;
            private byte FILTERTIME;
            private byte TRIGGLETIME;
            private byte BUZZERTIME;
            private byte INTERNELTIME;
            public byte Addr
            {
                get { return DEVICEARRD; }
                set { DEVICEARRD = value; }
            }
            public byte Protocol
            {
                get { return RFIDPRO; }
                set { RFIDPRO = value; }
            }
            public byte Baud
            {
                get { return BAUDRATE; }
                set { BAUDRATE = value; }
            }
            public byte Workmode
            {
                get { return WORKMODE; }
                set { WORKMODE = value; }
            }
            public byte port
            {
                get { return INTERFACE; }
                set { INTERFACE = value; }
            }
            public byte wieggand
            {
                get { return WGSET; }
                set { WGSET = value; }
            }
            public byte Ant
            {
                get { return ANT; }
                set { ANT = value; }
            }
            public byte Region
            {
                get { return REGION; }
                set { REGION = value; }
            }
            public byte Channel
            {
                get { return CN; }
                set { CN = value; }
            }
            public byte Power
            {
                get { return RFIDPOWER; }
                set { RFIDPOWER = value; }
            }
            public byte Area
            {
                get { return INVENTORYAREA; }
                set { INVENTORYAREA = value; }
            }
            public byte Q
            {
                get { return QVALUE; }
                set { QVALUE = value; }
            }
            public byte Session
            {
                get { return SESSION; }
                set { SESSION = value; }
            }
            public byte Startaddr
            {
                get { return ACSADDR; }
                set { ACSADDR = value; }
            }
            public byte DataLen
            {
                get { return ACSDATALEN; }
                set { ACSDATALEN = value; }
            }
            public byte Filtertime
            {
                get { return FILTERTIME; }
                set { FILTERTIME = value; }
            }
            public byte Triggletime
            {
                get { return TRIGGLETIME; }
                set { TRIGGLETIME = value; }
            }
            public byte Buzzertime
            {
                get { return BUZZERTIME; }
                set { BUZZERTIME = value; }
            }
            public byte IntenelTime
            {
                get { return INTERNELTIME; }
                set { INTERNELTIME = value; }
            }

            public ushort StartFreq
            {
                get
                {

                    return (ushort)(STRATFREI >> 8 | STRATFREI << 8);

                }
                set
                {
                    STRATFREI = (ushort)(value >> 8 | value << 8);          //大小端转换
                }
            }
            public ushort StartFreqde
            {
                get { return (ushort)(STRATFRED >> 8 | STRATFRED << 8); }
                set
                {
                    STRATFRED = (ushort)(value >> 8 | value << 8);          //大小端转换
                }
            }
            public ushort Stepfreq
            {
                get { return (ushort)(STEPFRE >> 8 | STEPFRE << 8); }
                set
                {
                    STEPFRE = (ushort)(value >> 8 | value << 8);          //大小端转换 
                }
            }
        };
    
}
