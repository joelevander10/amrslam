using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Windows.Forms;

namespace WindowsFormsApp1
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }
        private static readonly int FLAG_IN_INVENTORY = BitVector32.CreateMask();
        private static readonly int FLAG_STOP_INVENTORY = BitVector32.CreateMask(FLAG_IN_INVENTORY);

        public Reader Reader
        {
            get { return m_reader; }
        }

        // 标识集合
        BitVector32 m_flags = new BitVector32();

        // 停止盘点等待的时间
        private static readonly ushort s_usStopInventoryTimeout = 10000;
        // 盘点类型
        private byte m_btInvType = 0;
        // 盘点类型参数
        private uint m_uiInvParam = 0;

        // 是否正在显示标签数据，如果正在显示则为1，否则为0
        private int m_iShowingTag = 0;
        // 是否只更新列，如果是则为1 ，如果不是则为0；表示有没有更新的EPC号码
        private int m_iShowRow = 0;

        /// <summary>
        /// 每页显示的行数
        /// </summary>
        int m_iPageLines = 30;
        /// <summary>
        /// 当前显示的行
        /// </summary>
        int m_iPageIndex = 0;

        // 标签数据，用于查找相同的标签项，只是用来查找是否有相同卡号
        Dictionary<byte[], ShowTagItem> m_tags = new Dictionary<byte[], ShowTagItem>(1024, new TagCodeCompare());
        // 标签数据，用于标签按接收次序排序
        List<ShowTagItem> m_tags2 = new List<ShowTagItem>(1024);
        // 标签盘点响应总个数
        int m_iInvTagCount = 0;
        // 标签盘点总时间
        int m_iInvTimeMs = 1;
        // 开始盘点的时间
        int m_iInvStartTick = 0;
        // 盘点线程
        Thread m_thInventory = null;
        volatile bool m_bClosed = false;
        Reader m_reader = new Reader();
        Devicepara devicepara = new Devicepara();


        protected bool InInventory
        {
            get { return m_flags[FLAG_IN_INVENTORY]; }
            set { m_flags[FLAG_IN_INVENTORY] = value; }
        }

        public bool StopInventory
        {
            get { return m_flags[FLAG_STOP_INVENTORY]; }
            set { m_flags[FLAG_STOP_INVENTORY] = value; }
        }

        public bool IsClosed
        {
            get { return m_bClosed; }
        }


        private delegate void WriteLogHandler(MessageType type, string msg, Exception ex);
        public void WriteLog(MessageType type, string msg, Exception ex)
        {
            try
            {
                if (this.InvokeRequired)
                {
                    this.BeginInvoke(new WriteLogHandler(WriteLog), type, msg, ex);
                    return;
                }

                StringBuilder sb = new StringBuilder(128);
                sb.Append(DateTime.Now);
                sb.Append(", ");
                switch (type)
                {
                    case MessageType.Info:
                        sb.Append("info：");
                        break;
                    case MessageType.Warning:
                        sb.Append("warning：");
                        break;
                    case MessageType.Error:
                        sb.Append("error：");
                        break;
                }
                if (msg.Length > 0)
                    sb.Append(msg);
                if (ex != null)
                    sb.Append(ex.Message);
                sb.Append("\r\n");

                string msg2 = sb.ToString();
                stdSerialData.AppendText(msg2);
                stdSerialData.SelectionLength = 0;
                stdSerialData.SelectionStart = stdSerialData.TextLength;
                stdSerialData.ScrollToCaret();
            }
            catch { }
        }


        private void btnSportOpen_Click(object sender, EventArgs e)
        {
            try
            {
                string port = cmbComPort.Text.Trim();
                if (port.Length == 0)
                {
                    MessageBox.Show(this, "Failed to open the serial port, please enter the serial port number", this.Text);
                    return;
                }
                if (m_reader.IsOpened)
                {
                    MessageBox.Show(this, "The reader is already open, please close the reader first", this.Text);
                    return;
                }
                //serialport.DataReceived += serialport_DataReceived;
                m_reader.Open(port, (byte)cmbComBaud.SelectedIndex);

                WriteLog(MessageType.Info, "The reader is opened successfully, the serial port number：" + port, null);
                //WriteLog(MessageType.Info, "阅读器打开成功，串口号：" + port, null);
                cmbComPort.Enabled = false;
                btnSportOpen.Enabled = false;

                groupusb.Enabled = false;
                groupNet.Enabled = false;
            }
            catch (Exception ex)
            {
                try
                {
                    Reader reader = this.Reader;
                    if (reader.IsOpened)
                        reader.Close();
                }
                catch { }

                WriteLog(MessageType.Error, "Reader failed to open", ex);
                cmbComPort.Enabled = true;
                btnSportOpen.Enabled = true;
                MessageBox.Show(this, "Reader failed to open：" + ex.Message, this.Text);
            }
            InitReader();
        }
        private void InitReader()
        {
            try
            {
                Reader reader = this.Reader;
                if (reader.IsOpened)
                {

                    devicepara = reader.GetDevicePara();
                    cmbTxPower.Text = devicepara.Power.ToString();                           // 获取功率
                    if (devicepara.Workmode > 1)
                    {
                        cmbWorkmode.SelectedIndex = 0;//answer mode
                    }
                    else
                    {
                        cmbWorkmode.SelectedIndex = devicepara.Workmode; // active mode
                    }
                    cmbRegion.SelectedIndex = devicepara.Region;
                    btnGetOutInterface.PerformClick();
                    //btnGetAllPara.PerformClick();                                         // 需要enbable后才能虚拟click按键
                    //                                                                      //WriteLog(MessageType.Info,"Get device parameters", null);
                }
            }
            catch (Exception ex)
            {

                btnSportClose.PerformClick();
                btnDisConnect.PerformClick();

                WriteLog(MessageType.Error," Failed to get Power", ex);
                MessageBox.Show(this, "Failed to get power：" + ex.Message, this.Text);
                return;
            }
        }

        private void btnSportClose_Click(object sender, EventArgs e)
        {
            try
            {
                WriteLog(MessageType.Info, "Close Reader", null);
                cmbComPort.Enabled = true;
                btnSportOpen.Enabled = true;


                groupusb.Enabled = true;
                groupNet.Enabled = true;

                Reader reader = this.Reader;
                if (reader == null)
                    return;

                reader.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btnConnect_Click(object sender, EventArgs e)
        {
            try
            {
                string ip = txbIPAddr.Text.Trim();
                if (ip.Length == 0)
                {
                    MessageBox.Show(this, "Connect Failed，please input IP address", this.Text);
                    return;
                }
                System.Net.IPAddress ip2;
                if (!System.Net.IPAddress.TryParse(ip, out ip2))
                {
                    MessageBox.Show(this, "Error IPV4 Address", this.Text);
                    return;
                }
                string port = txbPort.Text.Trim();
                if (port.Length == 0)
                {
                    MessageBox.Show(this, "Please input port", this.Text);
                    return;
                }
                ushort port2;
                if (!ushort.TryParse(port, out port2) || port2 == 0)
                {
                    MessageBox.Show(this, "port num must 0~65535", this.Text);
                    return;
                }
                if (m_reader.IsOpened)
                {
                    MessageBox.Show(this, "reader is already opened ，please close first", this.Text);
                    return;
                }

                m_reader.Open(ip, port2, 3000, true);   // 3000ms等待时间

                WriteLog(MessageType.Info, "reader open success，IP address：" + ip2.ToString() + "，port：" + port2.ToString(), null);
                cmbComPort.Enabled = false;
                cmbComBaud.Enabled = false;
                txbIPAddr.Enabled = false;
                txbPort.Enabled = false;
                btnSportOpen.Enabled = false;
                btnConnect.Enabled = false;
                btnSportClose.Enabled = false;

                //Settings.Default.IPAddr = ip2.ToString();
                //Settings.Default.Port = port2.ToString();
                //Settings.Default.Save();

            }
            catch (Exception ex)
            {
                try
                {
                    Reader reader = this.Reader;
                    if (reader.IsOpened)
                        reader.Close();
                }
                catch { }

                WriteLog(MessageType.Error, "open reader fail", ex);
                cmbComPort.Enabled = true;
                cmbComBaud.Enabled = true;
                txbIPAddr.Enabled = true;
                txbPort.Enabled = true;
                btnSportOpen.Enabled = true;
                btnConnect.Enabled = true;
                btnSportClose.Enabled = true;
                MessageBox.Show(this, "open reader fail：" + ex.Message, this.Text);
            }
            InitReader();
        }

        private void btnDisConnect_Click(object sender, EventArgs e)
        {
            try
            {
                WriteLog(MessageType.Info, "Reader Disconnected", null);
                cmbComPort.Enabled = true;
                cmbComBaud.Enabled = true;
                txbIPAddr.Enabled = true;
                txbPort.Enabled = true;
                btnSportOpen.Enabled = true;
                btnConnect.Enabled = true;
                btnSportClose.Enabled = true;

                Reader reader = this.Reader;
                if (reader == null)
                    return;

                reader.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btnSetTxPower_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info,"Set device RfPower", null);

                byte power = (byte) Util.DecFromString(cmbTxPower.Text);
                devicepara.Power = power;
                reader.SetDevicePara(devicepara);
                WriteLog(MessageType.Info, "Set device RfPower Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btngetTxPower_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info, "Get device RfPower", null);
                devicepara = reader.GetDevicePara();
                cmbTxPower.Text = devicepara.Power.ToString();
                WriteLog(MessageType.Info, "Get device RfPower Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btnSetWorkMode_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info,"Set device Workmode", null);
                devicepara.Workmode = (byte)cmbWorkmode.SelectedIndex;
                WriteLog(MessageType.Info,"Set device Workmode Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btnGetWorkMode_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info, "Get device Workmode", null);
                devicepara =  reader.GetDevicePara();
                cmbWorkmode.SelectedIndex = devicepara.Workmode;
                WriteLog(MessageType.Info, "Get device Workmode Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }



        private void CloseInventoryThread()
        {
            try
            {
                StopInventory = true;
                if (!m_thInventory.Join(4000))
                    m_thInventory.Abort();
            }
            catch { }
        }

        private void OnInventoryEnd()
        {
            InInventory = false;
            StopInventory = true;
            btnInventory.Enabled = true;
            WriteLog(MessageType.Info, "Inventory completed", null);
        }

        private void DoStopInventory()
        {
            try
            {
                InInventory = false;
                StopInventory = true;
                btnInventory.Enabled = true;
                try
                {
                    Reader reader = this.Reader;
                    if (reader != null)
                    {
                            reader.InventoryStop(s_usStopInventoryTimeout);
                    }
                }
                catch (Exception) { };
            }
            catch { }
            try
            {
                this.BeginInvoke(new ThreadStart(OnInventoryEnd));
            }
            catch { }
        }

        private void UpdatePageIndex()
        {
            int count = m_tags.Count;
            int pageCount = count / m_iPageLines;
            if (count == 0)
            {
                m_iPageIndex = 0;
                return;
            }

            if (m_iPageIndex < 0)
                m_iPageIndex = 0;
            else if (m_iPageIndex > count)
                m_iPageIndex = count - 1;
        }



        private void ShowTag()
        {
            try
            {
                if (this.InvokeRequired)
                {
                    // 如果当前已经正在显示标签，则不用再次调用ShowTag
                    if (Interlocked.Exchange(ref m_iShowingTag, 1) == 1)
                        return;

                    // 显示标签
                    this.BeginInvoke(new ThreadStart(ShowTag));
                    return;
                }

                ListViewItem[] lvItems;
                int totalCount;
                int tagCount;
                int pageIndex;
                int pageLines;
                int totalTimeMs;
                lock (m_tags)
                {
                    UpdatePageIndex();
                    if (m_iPageLines == 0)
                    {
                        Interlocked.Exchange(ref m_iShowingTag, 0);
                        return;
                    }
                    tagCount = m_tags2.Count;
                    totalCount = m_iInvTagCount;
                    totalTimeMs = m_iInvTimeMs;
                    pageIndex = m_iPageIndex;
                    pageLines = m_iPageLines;

                    int index = m_iPageIndex;
                    int count = tagCount - m_iPageIndex;
                    lvItems = new ListViewItem[m_iPageLines > count ? count : m_iPageLines];
                    int i = 0;
                    for (; i < m_iPageLines && index < tagCount; i++, index++)
                    {
                        ShowTagItem sitem = m_tags2[index];
                        ListViewItem lvItem = new ListViewItem((index + 1).ToString());
                        lvItem.Tag = sitem;
                        lvItem.SubItems.Add(Util.HexArrayToString(sitem.Code));
                        lvItem.SubItems.Add(sitem.LEN.ToString());    //显示长度不显示信道
                        lvItem.SubItems.Add(sitem.CountsToString());
                        lvItem.SubItems.Add((sitem.Rssi / 10).ToString());
                        lvItem.SubItems.Add(sitem.Channel.ToString());
                        lvItems[i] = lvItem;
                    }
                }

                if (lvItems.Length != lsvTagsActive.Items.Count)
                {
                    lsvTagsActive.Items.Clear();
                    lsvTagsActive.Items.AddRange(lvItems);
                }
                else
                {
                    //lsvTags.BeginUpdate();

                    for (int i = 0; i < lvItems.Length; i++)             //只对第三列进行刷新
                    {
                        lsvTagsActive.Items[i].SubItems[2].Text = lvItems[i].SubItems[2].Text;          // count
                        lsvTagsActive.Items[i].SubItems[3].Text = lvItems[i].SubItems[3].Text;          // Rssi
                        lsvTagsActive.Items[i].SubItems[4].Text = lvItems[i].SubItems[4].Text;          // channel

                    }
                    //lsvTags.EndUpdate();

                }
                Interlocked.Exchange(ref m_iShowingTag, 0);
            }
            catch (Exception ex)
            {
                try { Interlocked.Exchange(ref m_iShowingTag, 0); }
                catch { }
                WriteLog(MessageType.Error, "An error occurred while displaying the label：", ex);
            }
        }



        /// <summary>
        /// 盘点线程主函数
        /// </summary>
        private void InventoryThread()
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                {
                    DoStopInventory();
                    return;
                }

                lock (m_tags)
                {
                    m_tags.Clear();
                    m_tags2.Clear();
                }

                ShowTag();

                m_iInvTagCount = 0;
                m_iInvStartTick = Environment.TickCount;

                while (!StopInventory)
                {
                    TagItem item;             //接收标签数据
                    try
                    {
                            item = reader.GetTagUii(1000);

                    }
                    catch (ReaderException ex)
                    {
                        if (ex.ErrorCode == ReaderException.ERROR_CMD_COMM_TIMEOUT ||
                            ex.ErrorCode == ReaderException.ERROR_CMD_RESP_FORMAT_ERROR)
                        {
                            if (reader != null && this.IsClosed)
                            {
                                DoStopInventory();
                                return;
                            }
                            continue;
                        }
                        throw ex;
                    }
                    if (item == null)     // 为空 表示周围没有标签或者指令结束
                        break;

                    if (item.Antenna == 0 || item.Antenna > 4)
                        continue;
                    //m_test_flag++;
                    lock (m_tags)                                           // 加锁，防止被其他线程改变
                    {
                        ShowTagItem sitem;                                  // 一个标签的结构体
                        if (m_tags.TryGetValue(item.Code, out sitem))        //判断是否已经盘点出来 根据EPC号码，sitem的值从m_tag中取出
                        {
                            sitem.IncCount(item);                            // 转换成 ShowTagItem 结构体，并保存，这里m_tag2为什么会更新？？
                                                                             //Console.WriteLine("sitem：" + sitem.Counts);
                                                                             //Console.WriteLine("mtag2：" + m_tags2[0].Counts);
                            m_iShowRow = 1;
                        }
                        else
                        {
                            sitem = new ShowTagItem(item);
                            m_tags.Add(item.Code, sitem);                  // 保存到字典
                            m_tags2.Add(sitem);                            // 保存到列表
                            m_iShowRow = 0;
                        }
                        m_iInvTagCount++;
                        m_iInvTimeMs = Environment.TickCount - m_iInvStartTick + 1;
                    }
                    ShowTag();
                }
                ShowTag();   //将上下位机的时序差导致的未显示的标签显示
                //WriteLog(MessageType.Info, "标签数="+ m_test_flag, null);
                this.BeginInvoke(new ThreadStart(OnInventoryEnd));
            }
            catch (Exception ex)
            {
                try
                {
                    WriteLog(MessageType.Error, "Inventory label failed：", ex);
                }
                catch { }
                DoStopInventory();
            }
        }


        private void btnInventoryActive_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");

                if (cmbWorkmode.SelectedIndex == 0)//应答模式,需要下发数据
                {
                    if (InInventory)
                    {
                        StopInventory = true;
                        CloseInventoryThread();
                        reader.InventoryStop(s_usStopInventoryTimeout);
                        btnInventory.Enabled = true;
                        return;
                    }
                    devicepara.Workmode = (byte)cmbWorkmode.SelectedIndex;
                    reader.SetDevicePara(devicepara);
                    WriteLog(MessageType.Info, "Set parameters successfully:", null);

                    m_btInvType = 0;
                    m_uiInvParam = 0;
                    btnInventory.Enabled = false;
                    WriteLog(MessageType.Info, "Start  inventory", null);
                    InInventory = true;
                    StopInventory = false;
                    System.Threading.Thread.Sleep(100);            // 延时函数 单位ms

                    reader.Inventory(m_btInvType, m_uiInvParam);   // start inventory
                    m_thInventory = new Thread(InventoryThread);
                    m_thInventory.Start();

                }
                else//主动模式
                {

                    if (InInventory)
                    {
                        StopInventory = true;
                        CloseInventoryThread();
                        btnInventory.Enabled = true;
                        return;
                    }
                    devicepara.Workmode = (byte)cmbWorkmode.SelectedIndex;
                    reader.SetDevicePara(devicepara);

                    btnInventory.Enabled = false;
                    InInventory = true;
                    StopInventory = false;

                    m_thInventory = new Thread(InventoryThread);
                    m_thInventory.Start();
                }

            }
            catch (Exception ex)
            {
                InInventory = false;
                StopInventory = true;
                btnInventory.Enabled = true;

                WriteLog(MessageType.Error, "Inventory label failed：", ex);
                MessageBox.Show(this, "Inventory label failed：" + ex.Message, "Tips");
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            m_bClosed = true;
            StopInventory = true;
        }

        private bool IsPortsChanged(string[] ports)
        {
            ComboBox.ObjectCollection items = cmbComPort.Items;
            if (items.Count != ports.Length)
                return true;

            for (int i = 0; i < ports.Length; i++)
            {
                if (string.Compare(items[i].ToString(), ports[i], StringComparison.OrdinalIgnoreCase) != 0)
                    return true;
            }
            return false;
        }

        private void cmbComPort_DropDown(object sender, EventArgs e)
        {
            try
            {
                string[] ports = SerialPort.GetPortNames();
                Array.Sort(ports);
                if (IsPortsChanged(ports))
                {
                    cmbComPort.Items.Clear();
                    cmbComPort.Items.AddRange(ports);
                }

            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btnClear_Click(object sender, EventArgs e)
        {
            lsvTagsActive.Items.Clear();
        }

        private void btninvStop_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");


                if (InInventory)
                {
                    StopInventory = true;
                    CloseInventoryThread();
                    if (devicepara.Workmode == 0)   // Ans mode
                    {
                        reader.InventoryStop(s_usStopInventoryTimeout);
                    }
                    btnInventory.Enabled = true;
                    return;
                }
                WriteLog(MessageType.Error, "Inventory Stoped", null);

            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            cmbComBaud.SelectedIndex = 4;  //default 115200 
        }

        private void cmbFreqStart_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void btnSetFreq_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info, "Set device Freq", null);

                devicepara.Region = (byte)cmbRegion.SelectedIndex;
                FreqInfo info = new FreqInfo();

                ChannelRegionItem region = ChannelRegionItem.OptionFromValue(ChannelRegionItem.StringToRegion(cmbRegion.Text),true);
                if (region == null)
                    return;

                float f_endfreq;
                float freq2;
                int step2;
                int count2;
                if (region.Value == 0)          // custom
                {
                    string starfreq = cmbFreqStart.Text.Trim();
                    if (starfreq.Length == 0)
                        throw new Exception("Please enter the starting frequency");
                    string step = (500).ToString();
                    string endfreq = cmbFreqEnd.Text.Trim();

                    if (!float.TryParse(starfreq, out freq2) || freq2 < 840 || freq2 > 960)
                        throw new Exception("The entered \"frequency\" value must be a number between 840 and 960 (including 840 and 960) (can be a decimal)");
                    if (!float.TryParse(endfreq, out f_endfreq) || f_endfreq < 840 || f_endfreq > 960)
                        throw new Exception("The entered \"frequency\" value must be a number between 840 and 960 (including 840 and 960) (can be a decimal)");
                    step2 = Util.NumberFromString(step);
                    if (step2 < 0 || step2 > 2000)
                        throw new Exception("The entered \"Channel Spacing Frequency\" value must be an integer between 0 and 500 (Include an integer between 1 and 500)");

                    count2 = (int)(f_endfreq - freq2) / step2;
                    if (count2 < 1 || count2 > 50)
                        throw new Exception("The entered \"Channel Spacing Frequency\" value must be an integer between 1 and 50 (Include an integer between 1 and 50)");
                }
                else
                {
                    ChannelItem item = cmbFreqStart.SelectedItem as ChannelItem;
                    if (item == null)
                        throw new Exception("Please select a starting frequency");
                    freq2 = item.Freq;

                    ChannelItem item1 = cmbFreqEnd.SelectedItem as ChannelItem;
                    if (item == null)
                        throw new Exception("Please select a ending frequency");
                    count2 = cmbFreqEnd.SelectedIndex - cmbFreqStart.SelectedIndex+1;
                    f_endfreq = freq2 + (region.FreqStep) * count2;
                }

                float i;
                i = freq2 * 1000;                                                   // 不能动

                devicepara.StartFreq = (ushort)freq2;
                devicepara.StartFreqde = (ushort)(i - ((int)freq2) * 1000);       // 精度问题，浮点数运算会丢失
                devicepara.Stepfreq = (ushort)region.FreqStep;
                devicepara.Channel = (byte)count2;

                reader.SetDevicePara(devicepara);
                WriteLog(MessageType.Info, "Set device Freq Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btnGetFreq_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info, "Get device Freq", null);
                devicepara = reader.GetDevicePara();

                FreqInfo freq = new FreqInfo();
                freq.Region = devicepara.Region;
                freq.StartFreq = (float)devicepara.StartFreq + (float)devicepara.StartFreqde / 1000.0f;
                freq.StepFreq = (ushort)devicepara.Stepfreq;
                freq.Count = devicepara.Channel;
                ChannelRegionItem region = ChannelRegionItem.OptionFromValue((ChannelRegion)freq.Region, true);

                cmbFreqStart.Text = freq.StartFreq.ToString("F3");

                cmbFreqEnd.Text = (freq.StartFreq + freq.StepFreq * freq.Count).ToString("F3");

                switch (region.Value)
                {
                    case ChannelRegion.USA:
                        cmbRegion.Text = "USA";
                        break;
                    case ChannelRegion.China_1:
                        cmbRegion.Text = "China_1";
                        break;
                    case ChannelRegion.China_2:
                        cmbRegion.Text = "China_2";
                        break;
                    case ChannelRegion.Europe3:
                        cmbRegion.Text = "Europe3";
                        break;
                    case ChannelRegion.Korea:
                        cmbRegion.Text = "Korea";
                        break;
                    case ChannelRegion.Europe:
                        cmbRegion.Text = "Europe";
                        break;
                    case ChannelRegion.Japan:
                        cmbRegion.Text = "Japan";
                        break;
                    case ChannelRegion.Malaysia:
                        cmbRegion.Text = "Malaysia";
                        break;
                    default:
                        //rdbFreqCustom.Checked = true;
                        //cmbFreqStart.Text = freq.StartFreq.ToString("F3");
                        //txbChannelStep.Text = freq.StepFreq.ToString();
                        //cmbFreqEnd.Text = freq.Count.ToString();
                        return;
                }

                ChannelItem item1 = null;
                ChannelCount item2 = null;
                foreach (ChannelItem item in cmbFreqStart.Items)
                {
                    if (Math.Abs(item.Freq - freq.StartFreq) < 0.01)
                    {
                        item1 = item;
                        break;
                    }
                }
                if (item1 != null)
                {
                    foreach (ChannelCount item in cmbFreqEnd.Items)
                    {
                        if (item.Count == freq.Count)
                        {
                            item2 = item;
                            break;
                        }
                    }
                }
                cmbFreqStart.SelectedItem = item1;
                cmbFreqEnd.SelectedItem = item2;

                WriteLog(MessageType.Info, "Get device Freq Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void cmbRegion_SelectedValueChanged(object sender, EventArgs e)
        {
            try
            {
                cmbFreqStart.Items.Clear();
                cmbFreqEnd.Items.Clear();


                ChannelRegionItem region = ChannelRegionItem.OptionFromValue(ChannelRegionItem.StringToRegion(cmbRegion.Text), true);
                if (region == null)
                    return;

                if (region.Value == ChannelRegion.Custom)
                {
                    //label31.Text = "信道个数：";
                    cmbFreqStart.DropDownStyle = ComboBoxStyle.DropDown;
                    cmbFreqEnd.DropDownStyle = ComboBoxStyle.DropDown;
                }
                else
                {
                    //label31.Text = "结束频率：";
                    cmbFreqStart.DropDownStyle = ComboBoxStyle.DropDownList;
                    cmbFreqEnd.DropDownStyle = ComboBoxStyle.DropDownList;


                    cmbFreqStart.Items.AddRange(region.GetChannelItems());
                    cmbFreqEnd.Items.AddRange(region.GetChanelCounts());

                    if (cmbFreqStart.Items.Count > 0)
                        cmbFreqStart.SelectedIndex = 0;
                    if (cmbFreqEnd.Items.Count > 0)
                        cmbFreqEnd.SelectedIndex = cmbFreqEnd.Items.Count - 1;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void Close_Realy_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info, "Set device Close_Realy", null);
                byte time = 0;        // time means Close Second  ,0-alltime
                reader.Close_Relay(time);
                WriteLog(MessageType.Info, "Set device Close_Realy Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void Release_Realy_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info, "Set device Release_Realy", null);
                byte time = 0;        // time means Close Second  ,0-alltime
                reader.Release_Relay(time);
                WriteLog(MessageType.Info, "Set device Release_Realy Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btnScanUsb_Click(object sender, EventArgs e)
        {
            string strSN = "";
            byte[] arrBuffer = new byte[256];
            string flag = "";
            int iHidNumber = 0;
            UInt16 iIndex = 0;
            cbxusbpath.Items.Clear();
            iHidNumber = m_reader.CFHid_GetUsbCount();
            for (iIndex = 0; iIndex < iHidNumber; iIndex++)
            {
                m_reader.CFHid_GetUsbInfo(iIndex, arrBuffer);
                strSN = System.Text.Encoding.Default.GetString(arrBuffer).Replace("\0", "");
                flag = strSN.Substring(strSN.Length - 3);
                if (flag == "kbd")    //键盘
                {
                    cbxusbpath.Items.Add("\\Keyboard-can'topen");
                }
                else                    // HID
                {
                    cbxusbpath.Items.Add("\\USB-open");
                }
                strSN = "";                   //需要清零
                arrBuffer = new byte[256];   //需要清零
            }
            if (iHidNumber > 0)
                cbxusbpath.SelectedIndex = 0;
        }

        private void btnUSBopen_Click(object sender, EventArgs e)
        {
            try
            {
                if (m_reader.IsOpened)
                {
                    MessageBox.Show(this, "The reader is already open, please close the reader first", this.Text);
                    return;
                }
                //serialport.DataReceived += serialport_DataReceived;
                m_reader.Open((byte)cbxusbpath.SelectedIndex);     // 打开选中的路径

                WriteLog(MessageType.Info, "The reader is opened successfully, the usb number：" + cbxusbpath.SelectedIndex, null);
                //WriteLog(MessageType.Info, "阅读器打开成功，串口号：" + port, null);

                btnScanUsb.Enabled = false;
                btnUSBopen.Enabled = false;
                groupNet.Enabled = false;
                gpbCom.Enabled = false;
            }
            catch (Exception ex)
            {
                try
                {
                    Reader reader = this.Reader;
                    if (reader.IsOpened)
                        reader.Close();
                }
                catch { }

                WriteLog(MessageType.Error, "Reader failed to open", ex);
                cmbComPort.Enabled = true;
                btnSportOpen.Enabled = true;
                MessageBox.Show(this, "Reader failed to open：" + ex.Message, this.Text);
            }
            InitReader();
        }

        private void btnUSBClose_Click(object sender, EventArgs e)
        {
            WriteLog(MessageType.Info, "Reader Disconnected", null);
            cbxusbpath.Enabled = true;
            btnUSBopen.Enabled = true;
            groupNet.Enabled = true;
            gpbCom.Enabled = true;

            Reader reader = this.Reader;
            if (reader == null)
                return;

            reader.Close();
        }

        private void btnGetOutInterface_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info, "Get device interface", null);
                devicepara = reader.GetDevicePara();
                byte interport;
                interport = devicepara.port;
                if (interport == 0x80)
                {
                    cmbOutInterface.SelectedItem = "RS232";
                }
                else if (interport == 0x40)
                {
                    cmbOutInterface.SelectedItem = "RS485";
                }
                else if (interport == 0x20)
                {
                    cmbOutInterface.SelectedItem = "RJ45";
                }
                else if (interport == 0x10)
                {
                    cmbOutInterface.SelectedItem = "WiFi";
                }
                else if (interport == 0x01)
                {
                    cmbOutInterface.SelectedItem = "USB";
                }
                else if (interport == 0x02)
                {
                    cmbOutInterface.SelectedItem = "KeyBoard";
                }
                else if (interport == 0x04)
                {
                    cmbOutInterface.SelectedItem = "CDC_COM";
                }
                WriteLog(MessageType.Info, "Get device interface Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }

        private void btnSetOutInterface_Click(object sender, EventArgs e)
        {
            try
            {
                Reader reader = this.Reader;
                if (reader == null)
                    throw new Exception("Reader not connected");
                WriteLog(MessageType.Info, "Set device OutInterface", null);
                devicepara.wieggand = 0x00;
                devicepara.port = (byte)cmbOutInterface.SelectedIndex;
                if (devicepara.port == 0)
                {
                    devicepara.port = 0x80;
                }
                else if (devicepara.port == 1)
                {
                    devicepara.port = 0x40;
                }
                else if (devicepara.port == 2)
                {
                    devicepara.port = 0x20;
                }
                else if (devicepara.port == 4)   // Wifi
                {
                    devicepara.port = 0x10;
                }
                else if (devicepara.port == 5)   // USB
                {
                    devicepara.port = 0x01;
                }
                else if (devicepara.port == 6)   // Keyoard
                {
                    devicepara.port = 0x02;
                }
                else if (devicepara.port == 7)   // CDC_COM
                {
                    devicepara.port = 0x04;
                }
                else
                {
                    devicepara.port = 0x80;
                }
                reader.SetDevicePara(devicepara);
                WriteLog(MessageType.Info, "Set device OutInterface Success", null);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, this.Text);
            }
        }
    }
}
