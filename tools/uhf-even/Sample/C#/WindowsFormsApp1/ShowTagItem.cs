using System;
using System.Collections.Generic;
using System.Text;
using System.Windows.Forms;

namespace WindowsFormsApp1
{
    /// <summary>
    /// 盘点到的标签项
    /// </summary>
    public class TagItem
    {
        private static readonly byte[] s_arrEmpty = new byte[0];

        /// <summary>
        /// 标签序号
        /// </summary>
        private ushort m_no;
        /// <summary>
        /// 标签的PC或编码长度+编码头数据
        /// </summary>
        private byte[] m_pc;
        /// <summary>
        /// 标签的UII或编码数据
        /// </summary>
        private byte[] m_code;
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
        private byte[] m_crc;
        /// <summary>
        /// codelengh
        /// </summary>
        private byte m_len;

        /// <summary>
        /// 标签序号
        /// </summary>
        public ushort NO
        {
            get { return m_no; }
        }

        /// <summary>
        /// 标签的PC或编码长度+编码头数据
        /// </summary>
        public byte[] PC
        {
            get { return m_pc; }
        }

        /// <summary>
        /// 标签的UII或编码数据
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

        /// <summary>
        /// len
        /// </summary>
        public byte LEN
        {
            get { return m_len; }
        }

        public TagItem(ushort no, byte[] pc, byte codeLen, byte[] code, short rssi, byte ant, byte channel, byte[] crc, byte len)
        {
            if (pc == null)
                throw new ArgumentNullException("pc");
            if (crc == null)
                throw new ArgumentNullException("crc");
            if (pc.Length != 2)
                throw new ArgumentException("PC must be 2 Byte");
            if (codeLen > 0 && code == null)
                throw new ArgumentNullException("code");
            if (codeLen > code.Length)
                throw new ArgumentOutOfRangeException("codeLen", "codelen is over");

            m_no = no;
            m_pc = pc;
            m_rssi = rssi;
            m_ant = ant;
            m_channel = channel;
            m_crc = crc;
            m_len = len;
            if (codeLen > 0)
            {
                m_code = new byte[codeLen];
                Array.Copy(code, m_code, codeLen);
            }
            else
                m_code = s_arrEmpty;
        }

        internal TagItem(TagInfo info)
        {
            //if (info.PC == null)
            //    throw new ArgumentNullException("pc");
            //if (info.CRC == null)
            //    throw new ArgumentNullException("crc");
            //if (info.PC.Length != 2)
            //    throw new ArgumentException("PC must be 2 Byte");
            byte codeLen = info.CodeLength;
            if (codeLen > 0 && info.Code == null)
                throw new ArgumentNullException("code");
            if (codeLen > info.Code.Length)
                throw new ArgumentOutOfRangeException("codeLen", "codelen is over");

            //m_no = info.NO;
            //m_pc = info.PC;
            m_len = info.CodeLength;         // 数据长度
            m_rssi = info.Rssi;
            m_ant = info.Antenna;
            m_channel = info.Channel;
            //m_crc = info.CRC;
            if (codeLen > 0)
            {
                m_code = new byte[codeLen];
                Array.Copy(info.Code, m_code, codeLen);
            }
            else
                m_code = s_arrEmpty;
        }

        public override int GetHashCode()
        {
            int hash = add(0, m_pc);
            hash = add(hash, m_crc);
            hash = add(hash, m_code);
            return hash;
        }

        public override bool Equals(object obj)
        {
            TagItem item = obj as TagItem;
            if (item == null)
                return false;
            return compare(m_crc, item.m_crc) &&
                compare(m_code, item.m_code) &&
                compare(m_pc, item.m_pc);
        }

        private bool compare(byte[] a, byte[] b)
        {
            if (a.Length != b.Length)
                return false;
            for (int i = 0; i < a.Length; i++)
            {
                if (a[i] != b[i])
                    return false;
            }
            return true;
        }

        private int add(int hash, byte[] data)
        {
            for (int i = 0; i < data.Length; i++)
            {
                if ((hash & 0x80000000) != 0)
                    hash = (hash << 1) + 1;
                else
                    hash <<= 1;
            }
            return hash;
        }
    }


    /// <summary>
    /// 盘点到的标签项
    /// </summary>
    public class ShowTagItem
    {
        private static readonly byte[] s_arrEmpty = new byte[0];

        /// <summary>
        /// 被盘点到的次数，每个天线分别统计
        /// </summary>
        private int[] m_counts;

        /// <summary>
        /// 标签序号
        /// </summary>
        private ushort m_no;
        /// <summary>
        /// 标签的PC或编码长度+编码头数据
        /// </summary>
        private byte[] m_pc;
        /// <summary>
        /// 标签的UII或编码数据
        /// </summary>
        private byte[] m_code;
        /// <summary>
        /// RSSI，单位：0.1dBm
        /// </summary>
        private short m_rssi;
        /// <summary>
        /// 信道，值范围：1~4
        /// </summary>
        private byte m_channel;
        /// <summary>
        /// CRC
        /// </summary>
        private byte[] m_crc;
        /// <summary>
        /// codelength
        /// </summary>
        private byte m_len;

        /// <summary>
        /// 标签序号
        /// </summary>
        public ushort NO
        {
            get { return m_no; }
        }

        /// <summary>
        /// 标签的PC或编码长度+编码头数据
        /// </summary>
        public byte[] PC
        {
            get { return m_pc; }
        }

        /// <summary>
        /// 标签的UII或编码数据
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

        /// <summary>
        /// LEN
        /// </summary>
        public byte LEN
        {
            get { return m_len; }
        }

        /// <summary>
        /// 被盘点到的次数
        /// </summary>
        public int[] Counts
        {
            get { return m_counts; }
        }

        public ShowTagItem(TagItem item)
        {
            if (item.Code == null)
                throw new ArgumentNullException("item.Code");
            if (item.Antenna == 0 || item.Antenna > 4)
                throw new ArgumentOutOfRangeException("item.Antenna");

            m_counts = new int[4];
            m_counts[item.Antenna - 1] = 1;
            m_no = item.NO;
            m_pc = item.PC;
            m_code = item.Code;
            m_rssi = item.Rssi;
            m_channel = item.Channel;
            m_crc = item.CRC;
            m_len = item.LEN;
        }

        public ShowTagItem(ushort no, byte[] pc, byte[] code, short rssi, byte ant, byte channel, byte[] crc,byte len)
        {
            if (pc == null)
                throw new ArgumentNullException("pc");
            if (pc.Length != 2)
                throw new ArgumentException("PC must be 2 Byte");
            if (code == null)
                throw new ArgumentNullException("code");
            if (ant == 0 || ant > 4)
                throw new ArgumentOutOfRangeException("channel");

            m_counts = new int[4];
            m_counts[ant - 1] = 1;
            m_no = no;
            m_rssi = rssi;
            m_channel = channel;
            m_crc = crc;
            m_code = code;
            m_len = len;
        }

        /// <summary>
        /// 获取被盘点到的次数
        /// </summary>
        /// <returns></returns>
        public string CountsToString()
        {
            StringBuilder sb = new StringBuilder(64);
            for (int i = 0; i < m_counts.Length; i++)
            {
                sb.Append(m_counts[i]);
                sb.Append('/');
            }
            sb.Length -= 1;
            return sb.ToString();
        }

        public void IncCount(TagItem item)
        {
            if (item.Antenna > 0 && item.Antenna <= 4)
                m_counts[item.Antenna - 1]++;

            m_no = item.NO;
            m_pc = item.PC;
            m_code = item.Code;
            m_rssi = item.Rssi;
            m_channel = item.Channel;
            m_crc = item.CRC;
            m_len = item.LEN;
        }

        public bool CompareCode(byte[] code)
        {
            if (code == null)
                return false;
            if (m_code == code)
                return true;
            if (m_code.Length != code.Length)
                return false;
            for (int i = 0; i < code.Length; i++)
            {
                if (m_code[i] != code[i])
                    return false;
            }
            return true;
        }

        public override string ToString()
        {
            return Util.HexArrayToString(m_code);
        }
    }

    class TagCodeCompare : IEqualityComparer<byte[]>
    {
        public bool Equals(byte[] x, byte[] y)
        {
            if (x == y)
                return true;
            if (x == null || y == null)
                return false;

            if (x.Length != y.Length)
                return false;
            for (int i = 0; i < x.Length; i++)
            {
                if (x[i] != y[i])
                    return false;
            }
            return true;
        }

        public int GetHashCode(byte[] obj)
        {
            if (obj == null)
                return 0;
            if (obj.Length == 0)
                return 0;
            int hash = 17;
            for (int i = 0; i < obj.Length; i++)
            {
                hash = (hash << 1) + obj[i];
            }
            return hash;
        }
    }
}
