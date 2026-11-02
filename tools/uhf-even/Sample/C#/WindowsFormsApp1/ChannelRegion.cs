using System;
using System.Collections.Generic;
using System.Text;

namespace WindowsFormsApp1
{
    /// <summary>
    /// Radio regulation region
    /// </summary>
    enum ChannelRegion
    {
        Custom = 0x00,
        USA,
        Korea,
        Europe,
        Japan,
        Malaysia,
        Europe3,
        China_1,
        China_2

    }

    class ChannelItem
    {
        public static readonly ChannelItem[] EmptyArray = new ChannelItem[0];

        float m_fFreq = 0;

        public float Freq
        {
            get { return m_fFreq; }
        }

        public ChannelItem(float freq)
        {
            m_fFreq = freq;
        }

        public override string ToString()
        {
            return m_fFreq.ToString("F3");
        }
    }

    class ChannelCount
    {
        public static readonly ChannelCount[] EmptyArray = new ChannelCount[0];

        int m_iCount = 0;
        ChannelRegionItem m_region = null;

        public int Count
        {
            get { return m_iCount; }
            set { m_iCount = value; }
        }

        public ChannelCount(int count, ChannelRegionItem region)
        {
            m_iCount = count;
            m_region = region;
        }

        public override string ToString()
        {
            if (m_region.Value == ChannelRegion.Custom)
                return m_iCount.ToString();
            return (m_region.StartFreq + (m_iCount - 1) * m_region.FreqStep / 1000f).ToString("F3");
        }
    }


    class ChannelRegionItem
    {
        public static readonly ChannelRegionItem USARegion = new ChannelRegionItem(ChannelRegion.USA, 902.750f, 500, 50);
        public static readonly ChannelRegionItem KoreaRegion = new ChannelRegionItem(ChannelRegion.Korea, 917.100f, 200, 32);
        public static readonly ChannelRegionItem EuropeRegion = new ChannelRegionItem(ChannelRegion.Europe, 865.100f, 200, 15);
        public static readonly ChannelRegionItem JapanRegion = new ChannelRegionItem(ChannelRegion.Japan, 952.200f, 200, 8);
        public static readonly ChannelRegionItem MalaysiaRegion = new ChannelRegionItem(ChannelRegion.Malaysia, 919.500f, 500, 7);
        public static readonly ChannelRegionItem Europe3Region = new ChannelRegionItem(ChannelRegion.Europe3, 865.700f, 600, 4);
        public static readonly ChannelRegionItem China1Region = new ChannelRegionItem(ChannelRegion.China_1, 840.125f, 250, 20);
        public static readonly ChannelRegionItem CustomRegion = new ChannelRegionItem(ChannelRegion.Custom, 0f, 0, 0);
        public static readonly ChannelRegionItem China2Region = new ChannelRegionItem(ChannelRegion.China_2, 920.125f, 250, 20);

        public static readonly ChannelRegionItem[] Options = new ChannelRegionItem[] {
            USARegion,
            KoreaRegion,
            EuropeRegion,
            JapanRegion,
            MalaysiaRegion,
            Europe3Region,
            China1Region,
            China2Region,
            CustomRegion
        };

        public static readonly ChannelRegionItem[] OptionsStandard = new ChannelRegionItem[] {
            USARegion,
            KoreaRegion,
            EuropeRegion,
            JapanRegion,
            MalaysiaRegion,
            Europe3Region,
            China1Region,
            China2Region,
            CustomRegion
        };

        ChannelRegion m_value;
        float m_fFreqStart;
        int m_iFreqStep;
        int m_iFreqCount;

        /// <summary>
        /// 区域
        /// </summary>
        public ChannelRegion Value
        {
            get { return m_value; }
        }

        /// <summary>
        /// 起始频率
        /// </summary>
        public float StartFreq
        {
            get { return m_fFreqStart; }
        }

        /// <summary>
        /// 信道频率间隔
        /// </summary>
        public int FreqStep
        {
            get { return m_iFreqStep; }
        }

        public ChannelRegionItem(ChannelRegion value, float freqStart, int freqStep, int freqCount)
        {
            m_value = value;
            m_fFreqStart = freqStart;
            m_iFreqStep = freqStep;
            m_iFreqCount = freqCount;
        }

        public ChannelItem[] GetChannelItems()
        {
            if (m_iFreqCount == 0)
                return ChannelItem.EmptyArray;
            float freq = m_fFreqStart;
            ChannelItem[] items = new ChannelItem[m_iFreqCount];
            for (int i = 0; i < items.Length; i++)
            {
                items[i] = new ChannelItem(freq);
                freq += m_iFreqStep / 1000f;
            }
            return items;
        }

        public ChannelCount[] GetChanelCounts()
        {
            if (m_iFreqCount == 0)
                return ChannelCount.EmptyArray;
            ChannelCount[] items = new ChannelCount[m_iFreqCount];
            for (int i = 0; i < items.Length; i++)
            {
                items[i] = new ChannelCount(i + 1, this);
            }
            return items;
        }

        public override string ToString()
        {
            return RegionToString(m_value);
        }

        public static string RegionToString(ChannelRegion value)
        {
            switch (value)
            {
                case ChannelRegion.Korea:
                    return "Korea";
                case ChannelRegion.Europe:
                    return "Europe";
                case ChannelRegion.China_1:
                    return "China_1";
                case ChannelRegion.China_2:
                    return "China_2";
                case ChannelRegion.USA:
                    return "USA";
                case ChannelRegion.Japan:
                    return "Japan";
                case ChannelRegion.Malaysia:
                    return "Malaysia";
                case ChannelRegion.Europe3:
                    return "Europe3";
                case ChannelRegion.Custom:
                    return "Custom";
            }
            return "未定义值：0x" + ((int)value).ToString("X2");
        }

        public static ChannelRegion StringToRegion(string value)
        {
            switch (value)
            {
                case "Korea":
                    return ChannelRegion.Korea;
                case "Europe":
                    return ChannelRegion.Europe;
                case "China_1":
                    return ChannelRegion.China_1;
                case "China_2":
                    return ChannelRegion.China_2;
                case "USA":
                    return ChannelRegion.USA;
                case "Japan":
                    return ChannelRegion.Japan;
                case "Malaysia":
                    return ChannelRegion.Malaysia;
                case "Europe3":
                    return ChannelRegion.Europe3;
                case "Custom":
                    return ChannelRegion.Custom;
            }
            return ChannelRegion.USA;
        }


        public static ChannelRegionItem OptionFromValue(ChannelRegion region, bool bStandard)
        {
            ChannelRegionItem[] items = bStandard ? OptionsStandard : Options;
            foreach (ChannelRegionItem item in items)
            {
                if (item.Value == region)
                    return item;
            }
            return null;
        }
    }
}

