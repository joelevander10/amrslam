using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;
using System.IO;

namespace WindowsFormsApp1
{
    public enum MessageType
    {
        Info,
        Warning,
        Error
    }


    static class Util
    {
        static readonly ushort[] g_pwCrc16Table = {
            0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
            0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
            0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
            0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
            0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
            0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
            0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
            0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
            0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
            0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
            0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
            0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
            0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
            0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
            0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
            0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
            0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
            0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
            0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
            0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
            0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
            0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
            0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
            0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
            0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
            0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
            0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
            0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
            0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
            0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
            0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
            0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0 };


        public static readonly byte[] EmptyArray = new byte[0];

        public static int NumberFromString(string value)
        {
            if (value == null)
                throw new ArgumentNullException("value");
            value = value.Trim();

            if (value.Length == 0)
                throw new FormatException("无法将空字符串转换为整数");

            if (value.Length > 1 && value[0] == '0' && (value[1] == 'x' || value[1] == 'X'))
                return HexFromString(value);
            char ch = value[value.Length - 1];
            if (ch == 'h' || ch == 'H')
                return HexFromString(value);
            return DecFromString(value);
        }

        public static int DecFromString(string value)
        {
            if (value == null)
                throw new ArgumentNullException("value");
            if (value.Length == 0)
                throw new FormatException("无法将空字符串转换为整数");

            int value2;
            if (!int.TryParse(value, out value2))
                throw new FormatException("无法将十进制字符串 '" + value + "' 转换为整数");
            return value2;
        }

        public static int HexFromString(string value)
        {
            if (value == null)
                throw new ArgumentNullException("value");
            if (value.Length == 0)
                throw new FormatException("无法将空字符串转换为整数");

            int value2 = 0;
            // 检测状态，0：尚未找到数字、正负符号或"0x"符号；1：正在转换数字；2：数字已结束
            int numState = 0;
            bool neg = false;
            for (int i = 0; i < value.Length; i++)
            {
                char ch = value[i];
                if (char.IsWhiteSpace(ch))
                {
                    if (numState == 1)
                        numState = 2;
                    continue;
                }
                if (numState == 2)
                    throw new FormatException("无法将十六进制字符串 '" + value + "' 转换为整数");

                if (ch == '+' || ch == '-')
                {
                    if (numState != 0)
                        throw new FormatException("无法将十六进制字符串 '" + value + "' 转换为整数");
                    neg = (ch == '-');
                    numState = 1;
                    continue;
                }

                if ((ch == 'h' || ch == 'H'))
                {
                    if (numState != 1)
                        throw new FormatException("无法将十六进制字符串 '" + value + "' 转换为整数");
                    numState = 2;
                    continue;
                }

                if (ch >= '0' && ch <= '9')
                {
                    // 0x or 0X
                    if (i + 1 < value.Length && (value[i + 1] == 'x' || value[i + 1] == 'X'))
                    {
                        i++;
                        numState = 1;
                    }
                    else
                    {
                        value2 = value2 * 16 + ch - '0';
                        numState = 1;
                    }
                    continue;
                }
                if (ch >= 'A' && ch <= 'F')
                {
                    value2 = value2 * 16 + ch - 'A' + 10;
                    numState = 1;
                    continue;
                }
                if (ch >= 'a' && ch <= 'f')
                {
                    value2 = value2 * 16 + ch - 'a' + 10;
                    numState = 1;
                    continue;
                }
                throw new FormatException("无法将十六进制字符串 '" + value + "' 转换为整数");
            }

            if (numState == 0)
                throw new FormatException("无法将十六进制字符串 '" + value + "' 转换为整数");
            if (neg && value2 != 0)
                value2 *= -1;
            return value2;
        }

        public static string HexArrayToString(byte[] arrData)
        {
            return HexArrayToString(arrData, 0, arrData.Length);
        }

        public static string HexArrayToString(byte[] arrData, int index, int len)
        {
            if (len == 0)
                return string.Empty;
            if (len == 1)
                return arrData[0].ToString("X2");
            StringBuilder sb = new StringBuilder(len * 3);
            if (arrData.Length > 0)
            {
                len = index + len - 1;
                for (int i = index; i < len; ++i)
                {
                    sb.Append(arrData[i].ToString("X2"));
                    sb.Append(' ');
                }
                sb.Append(arrData[len].ToString("X2"));
            }
            return sb.ToString();
        }

        public static byte[] HexArrayFromString(string value)
        {
            if (string.IsNullOrEmpty(value))
                return EmptyArray;

            List<byte> lstHex = new List<byte>(1024);
            HexArrayFromString(value, lstHex);
            return lstHex.ToArray();
        }

        public static void HexArrayFromString(string value, List<byte> lstHex)
        {
            // 当前状态
            // ，0 表示当前字节还没数据，等待接收第一个4位数据
            // ，1 表示已经接收第一个4位数据，等待接收第二个4位数据
            int iState = 0;
            byte btCur = 0, btTmp = 0;
            foreach (char ch in value)
            {
                switch (iState)
                {
                    case 0:
                        if (IsSpec(ch))
                        {
                            continue;
                        }
                        if (!IsHex(ch))
                        {
                            throw new FormatException("错误的十六进制字符串'" + value + "'");
                        }
                        btCur = Char2Hex(ch);
                        iState = 1;
                        break;
                    case 1:
                        if (IsSpec(ch))
                        {
                            lstHex.Add(btCur);
                            iState = 0;
                            continue;
                        }
                        if (!IsHex(ch))
                        {
                            throw new FormatException("错误的十六进制字符串'" + value + "'");
                        }
                        btTmp = Char2Hex(ch);
                        btCur = (byte)((btCur << 4) + btTmp);
                        lstHex.Add(btCur);
                        iState = 0;
                        break;
                    default:
                        throw new FormatException("错误的十六进制字符串'" + value + "'");
                }
            }
            if (iState == 1)
            {
                lstHex.Add(btCur);
                iState = 0;
            }
        }

        public static string FormatHexString(string value, ref int pos)
        {
            if (string.IsNullOrEmpty(value))
                return string.Empty;

            StringBuilder sb = new StringBuilder();
            int pos2 = pos;
            byte btCur = 0;
            int i = 0;
            int l = 0;
            int k = 0;
            for (; i < value.Length; i++)
            {
                char ch = value[i];
                if (IsSpec(ch))
                {
                    if (l == 0)
                    {
                        k = i + 1;
                        continue;
                    }
                }
                else
                {
                    l++;
                    if (l < 3)
                        continue;
                }

                if (l == 1)
                {
                    btCur = Char2Hex(value[k]);
                    sb.Append(btCur.ToString("X"));
                    sb.Append(' ');

                    if (i >= pos2)
                    {
                        pos = sb.Length - 1;
                        pos2 = int.MaxValue;
                    }
                }
                else if (l == 2)
                {
                    btCur = Char2Hex(value[k]);
                    btCur = (byte)((btCur << 4) + Char2Hex(value[k + 1]));
                    sb.Append(btCur.ToString("X2"));
                    sb.Append(' ');

                    if (i >= pos2)
                    {
                        pos = sb.Length - 1;
                        pos2 = int.MaxValue;
                    }
                }
                else
                {
                    if (i == pos2 + 1)
                    {
                        btCur = Char2Hex(value[k]);
                        sb.Append(btCur.ToString("X"));
                        sb.Append(' ');
                        btCur = Char2Hex(value[k + 1]);
                        btCur = (byte)((btCur << 4) + Char2Hex(value[k + 2]));
                        sb.Append(btCur.ToString("X2"));
                        sb.Append(' ');

                        pos = sb.Length - 4;
                        pos2 = int.MaxValue;
                    }
                    else if (i == pos2)
                    {
                        btCur = Char2Hex(value[k]);
                        btCur = (byte)((btCur << 4) + Char2Hex(value[k + 1]));
                        sb.Append(btCur.ToString("X2"));
                        sb.Append(' ');
                        btCur = Char2Hex(value[k + 2]);
                        sb.Append(btCur.ToString("X"));
                        sb.Append(' ');

                        pos = sb.Length - 2;
                        pos2 = int.MaxValue;
                    }
                    else if (i + 1 == pos2)
                    {
                        btCur = Char2Hex(value[k]);
                        btCur = (byte)((btCur << 4) + Char2Hex(value[k + 1]));
                        sb.Append(btCur.ToString("X2"));
                        sb.Append(' ');
                        btCur = Char2Hex(value[k + 2]);
                        sb.Append(btCur.ToString("X"));
                        sb.Append(' ');

                        pos = sb.Length - 1;
                        pos2 = int.MaxValue;
                    }
                    else
                    {
                        btCur = Char2Hex(value[k]);
                        btCur = (byte)((btCur << 4) + Char2Hex(value[k + 1]));
                        sb.Append(btCur.ToString("X2"));
                        sb.Append(' ');
                        i--;
                    }
                }
                l = 0;
                k = i + 1;
            }
            if (l > 0)
            {
                if (l == 1)
                {
                    btCur = Char2Hex(value[k]);
                    sb.Append(btCur.ToString("X"));
                    sb.Append(' ');

                    if (i >= pos2)
                    {
                        pos = sb.Length - 1;
                        pos2 = int.MaxValue;
                    }
                }
                else if (l == 2)
                {
                    btCur = Char2Hex(value[k]);
                    btCur = (byte)((btCur << 4) + Char2Hex(value[k + 1]));
                    sb.Append(btCur.ToString("X2"));
                    sb.Append(' ');

                    if (i >= pos2)
                    {
                        pos = sb.Length - 1;
                        pos2 = int.MaxValue;
                    }
                }
            }
            else if (sb.Length > 0)
                sb.Length -= 1;
            return sb.ToString();
        }

        static byte Char2Hex(char ch)
        {
            byte btHex = 0;
            if (ch >= '0' && ch <= '9')
            {
                btHex = (byte)(ch - '0');
            }
            else if (ch >= 'A' && ch <= 'Z')
            {
                btHex = (byte)(ch - 'A' + 10);
            }
            else if (ch >= 'a' && ch <= 'z')
            {
                btHex = (byte)(ch - 'a' + 10);
            }
            return btHex;
        }

        static bool IsHex(char ch)
        {
            return (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F') || (ch >= 'a' && ch <= 'f');
        }

        static bool IsSpec(char ch)
        {
            return (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n');
        }

        public static ushort CalcCrc16(byte[] pbtBuf, int pos, int len)
        {
            ushort wCrc = 0;

            for (int i = 0; i < len; i++)
            {
                wCrc = (ushort)((wCrc << 8) ^ g_pwCrc16Table[((wCrc >> 8) ^ pbtBuf[pos + i])]);
            }
            return wCrc;
        }
    }


    class API
    {
        [DllImport("UHFPrimeReader.dll", CharSet = CharSet.Ansi)]
        public static extern int OpenDevice(out IntPtr m_hPort, string strComPort, byte Baudrate);

        [DllImport("UHFPrimeReader.dll", CharSet = CharSet.Ansi)]
        public static extern int OpenNetConnection(out IntPtr handler, string ip, ushort port, uint timeoutMs);

        [DllImport("UHFPrimeReader.dll", CharSet = CharSet.Ansi)]
        public static extern int OpenHidConnection(out IntPtr handler, ushort index);

        [DllImport("UHFPrimeReader.dll")]
        public static extern int CFHid_GetUsbCount();

        [DllImport("UHFPrimeReader.dll")]
        public static extern bool CFHid_GetUsbInfo(UInt16 iIndex, byte[] pucDeviceInfo);


        [DllImport("UHFPrimeReader.dll")]
        public static extern int CloseDevice(IntPtr m_hPort);

        [DllImport("UHFPrimeReader.dll")]
        public static extern int GetDevicePara(IntPtr m_hPort, out Devicepara devInfo);

        [DllImport("UHFPrimeReader.dll")]
        public static extern int SetDevicePara(IntPtr m_hPort, Devicepara devInfo);


        [DllImport("UHFPrimeReader.dll")]
        public static extern int SetRFPower(IntPtr hComm, byte power, byte reserved);


        [DllImport("UHFPrimeReader.dll")]
        public static extern int InventoryContinue(IntPtr hComm, byte invCount, uint invParam);

        [DllImport("UHFPrimeReader.dll")]
        public static extern int GetTagUii(IntPtr hComm, out TagInfo tag_info, ushort timeout);

        [DllImport("UHFPrimeReader.dll")]
        public static extern int InventoryStop(IntPtr hComm, ushort timeout);

        [DllImport("UHFPrimeReader.dll")]
        public static extern int Release_Relay(IntPtr hComm, byte time);

        [DllImport("UHFPrimeReader.dll")]
        public static extern int Close_Relay(IntPtr hComm, byte time);

    }

    public class Reader
    {
        private static readonly byte[] s_arrEmpty = new byte[0];

        // reader handel
        IntPtr m_handler = IntPtr.Zero;

        // reader open status，0：close；1：open by serial；2：open by net
        volatile int m_iState = 0;
        ushort m_usbindex = 0;

        string m_ip = string.Empty;
        ushort m_netPort = 0;
        string m_sport = string.Empty;




        static Reader s_reader = null;

        public bool IsOpened
        {
            get { return m_iState != 0; }
        }

        public bool IsOpenedAsCom
        {
            get { return m_iState == 1; }
        }

        public bool IsOpenedAsNetwork
        {
            get { return m_iState == 2; }
        }

        public string IPAddress
        {
            get { return m_ip; }
        }

        public ushort NetPort
        {
            get { return m_netPort; }
        }

        public string PortName
        {
            get { return m_sport; }
        }

        public void Open(string port, byte Baudrate)
        {
            if (port == null)
                throw new ArgumentNullException("port(Serial)");
            port = port.Trim();
            if (port.Length == 0)
                throw new ArgumentException("please input serial", "port(serial)");
            if (m_iState != 0)
                throw new InvalidOperationException("reader is already opened");

            if (m_handler != IntPtr.Zero)
            {
                try { API.CloseDevice(m_handler); }
                catch { }
                m_handler = IntPtr.Zero;
                s_reader = null;
            }
            int state = API.OpenDevice(out m_handler, port, Baudrate);
            if (state != ReaderException.ERROR_SUCCESS)
            {
                m_handler = IntPtr.Zero;
                s_reader = null;
                throw new IOException("serial '" + port + "' open fail");
            }
            s_reader = this;
            m_sport = port;
            m_iState = 1;
        }

        public void Open(string ip, ushort port, uint timeoutMs, bool throwExcpOnTimeout)
        {
            if (ip == null)
                throw new ArgumentNullException("ip(addr)");
            ip = ip.Trim();
            if (ip.Length == 0)
                throw new ArgumentException("please input IP addr", "ip(addr)");
            if (port == 0)
                throw new ArgumentException("port can not be 0", "port(port )");
            if (m_iState != 0)
                throw new InvalidOperationException("reader is already opened");

            if (m_handler != IntPtr.Zero)
            {
                try { API.CloseDevice(m_handler); }
                catch { }
                m_handler = IntPtr.Zero;
                s_reader = null;
            }

            int state = API.OpenNetConnection(out m_handler, ip, port, timeoutMs);
            if (state != ReaderException.ERROR_SUCCESS)
            {
                m_handler = IntPtr.Zero;
                s_reader = null;
                if (state == ReaderException.ERROR_CMD_COMM_TIMEOUT && !throwExcpOnTimeout)
                    return;
                throw new IOException("IP'" + ip + "' port " + port + " connecting fail");
            }
            s_reader = this;
            m_ip = ip;
            m_netPort = port;
            m_iState = 2;
        }

        public void Open(ushort index)
        {
            if (m_iState != 0)
                throw new InvalidOperationException("Reader is already open");

            if (m_handler != IntPtr.Zero)
            {
                try { API.CloseDevice(m_handler); }
                catch { }
                m_handler = IntPtr.Zero;
                s_reader = null;
            }

            int state = API.OpenHidConnection(out m_handler, index);
            if (state != ReaderException.ERROR_SUCCESS)
            {
                m_handler = IntPtr.Zero;
                s_reader = null;
                if (state == ReaderException.ERROR_CMD_COMM_TIMEOUT)
                    return;
            }
            s_reader = this;
            m_usbindex = index;
            m_iState = 3;
        }




        public void Close()
        {
            if (m_handler != IntPtr.Zero)
            {
                try { API.CloseDevice(m_handler); }
                catch { }
                m_handler = IntPtr.Zero;
                s_reader = null;
            }
            m_iState = 0;
        }


        // 0:未找到设备
        // 1:一个设备
        public int CFHid_GetUsbCount()
        {
            int count = API.CFHid_GetUsbCount();
            return count;
        }

        public bool CFHid_GetUsbInfo(UInt16 iIndex, byte[] pucDeviceInfo)
        {
            bool jet = API.CFHid_GetUsbInfo(iIndex, pucDeviceInfo);
            return jet;
        }



        public Devicepara GetDevicePara()
        {
            if (m_iState == 0)
                throw new InvalidOperationException("Reader is not open");
            Devicepara info;
            //DegbugPrint("Start GetVer");
            int state = API.GetDevicePara(m_handler, out info);
            //DegbugPrint("End GetVer");
            if (state == ReaderException.ERROR_SUCCESS)
                return info;
            throw new ReaderException(state);
        }

        public void SetDevicePara(Devicepara info)
        {
            if (m_iState == 0)
                throw new InvalidOperationException("Reader is not open");
            //DegbugPrint("Start GetVer");
            int state = API.SetDevicePara(m_handler, info);
            //DegbugPrint("End GetVer");
            if (state == ReaderException.ERROR_SUCCESS)
                return;
            throw new ReaderException(state);
        }

        public void SetRfTxPower(byte txPower, byte reserved)
        {
            if (m_iState == 0)
                throw new InvalidOperationException("Reader is not open");

            int state = API.SetRFPower(m_handler, txPower, reserved);
            if (state == ReaderException.ERROR_SUCCESS)
                return;
            throw new ReaderException(state);
        }



        public void Release_Relay(byte time)
        {
            if (m_iState == 0)
                throw new InvalidOperationException("Reader is not open");

            int state = API.Release_Relay(m_handler ,time);
            if (state == ReaderException.ERROR_SUCCESS)
                return;
            throw new ReaderException(state);
        }


        public void Close_Relay(byte time)
        {
            if (m_iState == 0)
                throw new InvalidOperationException("Reader is not open");

            int state = API.Close_Relay(m_handler, time);
            if (state == ReaderException.ERROR_SUCCESS)
                return;
            throw new ReaderException(state);
        }




        public void Inventory(byte invCount, uint invParam)
        {
            if (m_iState == 0)
                throw new InvalidOperationException("Reader is not open");

            int state = API.InventoryContinue(m_handler, invCount, invParam);
            if (state == ReaderException.ERROR_SUCCESS)
                return;
            throw new ReaderException(state);
        }



        public void InventoryStop(ushort timeoutMs)
        {
            if (m_iState == 0)
                throw new InvalidOperationException("Reader is not open");

            int state = API.InventoryStop(m_handler, timeoutMs);
            if (state == ReaderException.ERROR_SUCCESS)
                return;
            throw new ReaderException(state);
        }

        public TagItem GetTagUii(ushort timeoutMs)
        {
            if (m_iState == 0)
                throw new InvalidOperationException("Reader is not open");

            TagInfo info;
            int state = API.GetTagUii(m_handler, out info, timeoutMs);
            if (state == ReaderException.ERROR_CMD_NO_TAG)
                return null;
            if (state == ReaderException.ERROR_SUCCESS)
                return new TagItem(info);
            throw new ReaderException(state);
        }




        static readonly ushort PRESET_VALUE = 0xFFFF;
        static readonly ushort POLYNOMIAL = 0x8408;

        public unsafe ushort Crc16Cal(byte[] pucY, ushort ucX,ushort CrcValue)
        {
            ushort ucI, ucJ;
            ushort uiCrcValue;
            if (CrcValue == 0xFFFF)    // first value
            {
                uiCrcValue = PRESET_VALUE;
            }
            else {
                uiCrcValue = CrcValue;
            }
	        for (ucI = 1; ucI<ucX; ucI++)
	        {
		        uiCrcValue = (ushort)(uiCrcValue ^ pucY[ucI]);
		        for (ucJ = 0; ucJ< 8; ucJ++)
		        {
			        if ((uiCrcValue & 0x0001) != 0)
			        {
				        uiCrcValue = (ushort)((uiCrcValue >> 1) ^ POLYNOMIAL);
			        }
			        else
			        {
				        uiCrcValue = (ushort)(uiCrcValue >> 1);
			        }
		        }
	        }
	        return uiCrcValue;
        }

    }
}
