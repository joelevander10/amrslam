#include <cstdio>
#include <dlfcn.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>

#include "CFApi.h"

using namespace std;

void Read(int64_t hComm)
{
	unsigned char arrBuffer[2048] = { 0 };
	unsigned short iTagLength = 0;
	unsigned short iTagNumber = 0;
	TagInfo tag;
	int64_t result;
	char buf[512] = { 0 };

	result = GetTagUii(hComm, &tag, 3000);
	if (result == 0x00)
	{
		string str1 = "", str2 = "", strTemp = "";
		sprintf(buf, "Data:");
		strTemp = buf;
		str2 = str2 + strTemp;
		for (int i = 0;i < 12;i++) {
			sprintf(buf, "%.2X ", tag.code[i]);
			str1 = buf;
			str2 = str2 + str1;
		}
		sprintf(buf, " Len:%d", tag.codeLen);
		strTemp = buf;
		str2 = str2 + strTemp;
		sprintf(buf, " Ant:%.2X", tag.antenna);
		strTemp = buf;
		str2 = str2 + strTemp;
		sprintf(buf, " RSSI:%d", (int)tag.rssi / 10);
		strTemp = buf;
		str2 = str2 + strTemp;
		sprintf(buf, " Channel:%d", tag.channel);
		strTemp = buf;
		str2 = str2 + strTemp;

		cout << str2 << endl;
	}
}

int Operate(int64_t hComm) {

	int64_t result;
	cout << "=========================================================" << endl;
	cout << "[0]:GetRFPower" << endl;
	cout << "[1]:SetRFpower" << endl;
	cout << "[2]:GetWorkmode" << endl;
	cout << "[3]:SetWorkmode" << endl;
	cout << "[4]:CloseRealy" << endl;
	cout << "[5]:ReleaseRealy" << endl;
	cout << "[6]:Read 100 Data" << endl;

	cout << "Please input key" << endl;
	int key = 0;
	cin >> key;

	DevicePara param;
	string str;
	char buf[16] = { 0 };
	int power;
	int bValue;
	int workmode;
	string strWorkmode;

	int i = 0;
	switch (key) {
	case 0:
		// GetRFPower
		if (GetDevicePara(hComm, &param) != 0x00) {
			cout << "Get RFPower Failed" << endl;
			return 0;
		}
		power = (int)param.RFIDPOWER;
		cout << "The RFPower is " + to_string(power) << endl;
		break;
	case 1:
		// SetRFpower
		cout << "Please input RFValue" << endl;
		scanf("%d", &bValue);
		GetDevicePara(hComm, &param);
		param.RFIDPOWER = bValue;
		result = SetDevicePara(hComm, param);
		if (result != 0x00)
		{
			cout << "Set RFpower Failed" << endl;
			return 0;
		}
		cout << "Set RFpower Success" << endl;
		break;
	case 2:
		// GetWorkmode
		if (GetDevicePara(hComm, &param) != 0x00) {
			cout << "Get Workmode Failed" << endl;
			return 0;
		}
		workmode = (int)param.WORKMODE;
		switch (workmode)
		{
		case 0:
			strWorkmode = "Answer Mode";
			break;
		case 1:
			strWorkmode = "Active Mode";
			break;
		default:
			break;
		}
		cout << "The Workmode is " + strWorkmode << endl;
		break;
	case 3:
		// SetWorkmode
		cout << "Please input Workmode:" << endl;
		cout << "[0]Answer Mode" << endl;
		cout << "[1]Active Mode" << endl;
		scanf("%d", &bValue);
		GetDevicePara(hComm, &param);
		param.WORKMODE = bValue;
		result = SetDevicePara(hComm, param);
		if (result != 0x00)
		{
			cout << "Set Workmode Failed" << endl;
			return 0;
		}
		cout << "Set Workmode Success" << endl;
		break;
	case 4:
		// CloseRealy
		result = Close_Relay(hComm, 0);
		if (result != 0x00)
		{
			cout << "Close Realy Failed" << endl;
			return 0;
		}
		cout << "Close Relay Success" << endl;
		break;
	case 5:
		// ReleaseRealy
		result = Release_Relay(hComm, 0);
		if (result != 0x00)
		{
			cout << "Release Realy Failed" << endl;
			return 0;
		}
		cout << "Release Relay Success" << endl;
		break;
	case 6:
		// StartRead
		if (InventoryContinue(hComm, 0, 0) != 0x00)
		{
			cout << "Start Read Failed" << endl;
			return 0;
		}
		for (i = 0; i < 100;i++)
		{
			Read(hComm);
			usleep(10000);
		}
		if (InventoryStop(hComm, 10000) != 0x00)
		{
			cout << "Stop Read Failed" << endl;
			return 0;
		}
		break;
	default:
		break;
	}
	cout << "Enter 'E' to continue, Enter 'Q' to exit!" << endl;
	char method;
	while (1)
	{
		cin >> method;
		if (method == 'E' || method == 'e')
		{
			return Operate(hComm);
		}
		else if (method == 'Q' || method == 'q') {
			return 0;
		}
	}
}

int main()
{
	cout << "CF_CDemo" << endl;
	cout << "Please select the connection method" << endl;
	cout << "[a]COM" << endl;
	cout << "[b]USB" << endl;
	cout << "[c]TCP" << endl;
	char method;
	int64_t hComm;
	int64_t result;
	char strDev[256] = { 0 };
	int iPort = 0;
	int count;
	int index;
	cin >> method;
	switch (method)
	{
	case 'a':
	case 'A':
		cout << "OpenDevice:Please input device description like /dev/ttyS1" << endl;
		cin >> strDev;
		// result = OpenDevice(&hComm, "/dev/ttyS1", 115200);
		result = OpenDevice(&hComm, strDev, 115200);
		break;
	case 'b':
	case 'B':
		count = CFHid_GetUsbCount();
		for (int i = 0; i < count; i++)
		{
			char info[256];
			CFHid_GetUsbInfo(i, info);
			cout << info << endl;
		}
		index = 0;
		cin >> index;
		result = OpenHidConnection(&hComm, index);
		break;
	case 'c':
	case 'C':
		cout << "OpenDevice:Please input IP and port like 192.168.1.222 2022" << endl;
		cin >> strDev >> iPort;
		// result = OpenNetConnection(&hComm, "192.168.1.222", 2022, 1000);
		result = OpenNetConnection(&hComm, strDev, iPort, 1000);
		break;
	default:
		break;
	}

	if (result != 0x00)
	{
		printf("Open Failed!%.2X\r\n", result);
		return 0;
	}
	else {
		cout << "Open Success" << endl;

		return Operate(hComm);
	}
	return 0;
}