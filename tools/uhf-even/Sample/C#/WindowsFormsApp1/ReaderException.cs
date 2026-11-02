using System;
using System.Collections.Generic;
using System.Text;

namespace WindowsFormsApp1
{
    public class ReaderException : Exception
    {
        /// <summary>
        /// 执行成功
        /// </summary>
        public static readonly int ERROR_SUCCESS = 0x00;
        /// <summary>
        /// 句柄错误
        /// </summary>
        public static readonly int ERROR_HANDLE_ERROR = -255;
        /// <summary>
        /// 打开串口或建立网络连接失败
        /// </summary>
        public static readonly int ERROR_OPEN_FAILED = -254;
        /// <summary>
        /// 动态库内部错误
        /// </summary>
        public static readonly int ERROR_DLL_INNER_ERROR = -253;
        /// <summary>
        /// 参数值错误或越界，或者模块不支持该参数值
        /// </summary>
        public static readonly int ERROR_CMD_PARAM_ERROR = -252;
        /// <summary>
        /// 序列号已存在
        /// </summary>
        public static readonly int ERROR_CMD_SERIAL_NUM_EXISTS = -251;
        /// <summary>
        /// 由于模块内部错误导致命令执行失败
        /// </summary>
        public static readonly int ERROR_CMD_INTERNAL_ERROR = -250;
        /// <summary>
        /// 没有盘点到标签或盘点已结束
        /// </summary>
        public static readonly int ERROR_CMD_NO_TAG = -249;
        /// <summary>
        /// 标签响应超时
        /// </summary>
        public static readonly int ERROR_CMD_TAG_RESP_TIMEOUT = -248;
        /// <summary>
        /// 解调标签数据错误
        /// </summary>
        public static readonly int ERROR_CMD_TAG_RESP_COLLISION = -247;
        /// <summary>
        /// 标签数据超出最大传输长度
        /// </summary>
        public static readonly int ERROR_CMD_CODE_OVERFLOW = -246;
        /// <summary>
        /// 认证失败
        /// </summary>
        public static readonly int ERROR_CMD_AUTH_FAILED = -245;
        /// <summary>
        /// 口令错误
        /// </summary>
        public static readonly int ERROR_CMD_PWD_ERROR = -244;
        /// <summary>
        /// SAM卡无响应
        /// </summary>
        public static readonly int ERROR_CMD_SAM_NO_RESP = -243;
        /// <summary>
        /// PSAM卡命令执行失败
        /// </summary>
        public static readonly int ERROR_CMD_SAM_CMD_FAILED = -242;
        /// <summary>
        /// 读写器响应格式错误
        /// </summary>
        public static readonly int ERROR_CMD_RESP_FORMAT_ERROR = -241;
        /// <summary>
        /// 命令执行成功，但是还有后续数据未传输完成
        /// </summary>
        public static readonly int ERROR_CMD_HAS_MORE_DATA = -240;
        /// <summary>
        /// 传入缓存太小，数据溢出
        /// </summary>
        public static readonly int ERROR_CMD_BUF_OVERFLOW = -239;
        /// <summary>
        /// 等待阅读器响应超时
        /// </summary>
        public static readonly int ERROR_CMD_COMM_TIMEOUT = -238;
        /// <summary>
        /// 向读写器写数据失败
        /// </summary>
        public static readonly int ERROR_CMD_COMM_WRITE_FAILED = -237;
        /// <summary>
        /// 从读写器读数据失败
        /// </summary>
        public static readonly int ERROR_CMD_COMM_READ_FAILED = -236;
        /// <summary>
        /// 所有数据已传输完毕
        /// </summary>
        public static readonly int ERROR_CMD_NOMORE_DATA = -235;
        /// <summary>
        /// 网络连接尚未建立
        /// </summary>
        public static readonly int ERROR_DLL_UNCONNECT = -234;
        /// <summary>
        /// 网络连接已经断开
        /// </summary>
        public static readonly int ERROR_DLL_DISCONNECT = -233;
        /// <summary>
        /// 接收响应CRC校验错误
        /// </summary>
        public static readonly int ERROR_CMD_RESP_CRC_ERROR = -232;
        /// <summary>
        /// 下载数据校验错误
        /// </summary>
        public static readonly int ERROR_CMD_IAP_CRC_ERR = -231;
        /// <summary>
        /// 下载数据错误
        /// </summary>
        public static readonly int ERROR_CMD_DOWMLOAD_ERR = -230;
        /// <summary>
        /// 数据未下载成功
        /// </summary>
        public static readonly int ERROR_CMD_DOWM_NONE_ERR = -229;




        private static readonly ErrorItem[] s_arrErrs = new ErrorItem[] { 
         new ErrorItem(ERROR_SUCCESS, "Command executed successfully"),
         new ErrorItem(ERROR_HANDLE_ERROR, "Incorrect handle or parameter"),
         new ErrorItem(ERROR_OPEN_FAILED, "Failed to open the reader"),
         new ErrorItem(ERROR_DLL_INNER_ERROR, "Internal dynamic library error"),
         new ErrorItem(ERROR_CMD_RESP_FORMAT_ERROR, "The reader responds to a data format error"),
         new ErrorItem(ERROR_CMD_RESP_CRC_ERROR, "The reader responded to a CRC check error"),
         new ErrorItem(ERROR_CMD_BUF_OVERFLOW, "The incoming cache is too small and the data overflows"),
         new ErrorItem(ERROR_CMD_COMM_TIMEOUT, "Waiting for reader response timed out"),
         new ErrorItem(ERROR_CMD_COMM_WRITE_FAILED, "An error occurred writing data to the reader"),
         new ErrorItem(ERROR_CMD_COMM_READ_FAILED, "An error occurred while reading data from the reader"),
         new ErrorItem(ERROR_CMD_HAS_MORE_DATA, "Subsequent data transmission is not complete"),
         new ErrorItem(ERROR_DLL_UNCONNECT, "The network connection has not been established"),
         new ErrorItem(ERROR_DLL_DISCONNECT, "The network connection has been disconnected"),
         new ErrorItem(ERROR_CMD_IAP_CRC_ERR, "Download data verification error"),
         new ErrorItem(ERROR_CMD_DOWMLOAD_ERR, "Data download error, data write error"),
         new ErrorItem(ERROR_CMD_DOWM_NONE_ERR, "Data download failed. Procedure"),
 };

        /// <summary>
        /// 错误码
        /// </summary>
        private int m_iErroCode = 0;

        /// <summary>
        /// 错误码
        /// </summary>
        public int ErrorCode
        {
            get { return m_iErroCode; }
        }

        public ReaderException()
            : this(ERROR_SUCCESS)
        {
        }

        public ReaderException(int errorCode)
            : this(errorCode, MessageFromErrorCode(errorCode))
        {
        }

        public ReaderException(int errorCode, string message)
            : base(message)
        {
            this.m_iErroCode = errorCode;
        }

        public static string MessageFromErrorCode(int errorCode)
        {
            for (int i = 0; i < s_arrErrs.Length; i++)
            {
                if (s_arrErrs[i].ErrorCode == errorCode)
                    return s_arrErrs[i].Message;
            }
            return "Unknown error code：" + errorCode;
        }

        private struct ErrorItem
        {
            public int ErrorCode;
            public string Message;

            public ErrorItem(int errorCode, string message)
            {
                this.ErrorCode = errorCode;
                this.Message = message;
            }
        }
    }
}
