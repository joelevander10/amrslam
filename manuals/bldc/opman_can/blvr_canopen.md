# blvr_canopen


## Page 1

HP-5143-3
Brushless Motor
BLV series R type Driver
CANopen communication profile

## Page 2

Table of contents
1 General Information .............................................................................................8
1.1 About this manual ................................................................................................................8
1.2 Object description ................................................................................................................9
2 Installation and Setup .......................................................................................10
2.1 Installation and connection ............................................................................................10
2.2 Connection of termination resistor ..............................................................................11
2.3 Setting the Bitrate and Node-ID ....................................................................................11
3 Communication specifications ........................................................................12
3.1 CANopen device model ....................................................................................................12
3.2 General description of CAN .............................................................................................13
3.3 Communication Object Identifier (COB-ID) ...............................................................14
3.4 Endian format .......................................................................................................................15
4 Communication Objects ...................................................................................16
4.1 Network Management (NMT) ........................................................................................16
4.2 NMT Error Control ...............................................................................................................18
4.2.1 Node Guarding ..................................................................................................................18
4.2.2 Heartbeat .............................................................................................................................20
4.3 NMT boot-up ........................................................................................................................21
4.4 Synchronization object (SYNC) ......................................................................................21
4.5 Emergency object (EMCY) ...............................................................................................22
4.6 Service data object (SDO) ................................................................................................24
4.6.1 SDO download initiate ....................................................................................................26
4.6.2 SDO download segment ................................................................................................27
4.6.3 SDO upload initiate ..........................................................................................................28
4.6.4 SDO upload segment ......................................................................................................29
4.6.5 SDO abort transfer ............................................................................................................30
4.7 Process data object (PDO) ...............................................................................................31
4.7.1 PDO mapping .....................................................................................................................32
4.7.2 Transmission type .............................................................................................................32
5 Device control .....................................................................................................33
5.1 Status Machine ....................................................................................................................33
6 Status Machine control commands .................................................................34
6.1 Bits in Statusword (6041h) ...............................................................................................34
6.2 Transitions of the status machine .................................................................................35
6.3 Related Objects ....................................................................................................................35
7 Operation mode..................................................................................................36
7.1 Modes of Operation ...........................................................................................................36
7.1.1 General Information .........................................................................................................36
7.1.2 Related Objects..................................................................................................................36
7.2 Profile Velocity Mode (pv) ................................................................................................37
2

## Page 3

Table of contents
7.2.1 General Information .........................................................................................................37
7.2.2 Related Objects..................................................................................................................38
7.2.3 Controlword of the Profile Velocity Mode ................................................................38
7.2.4 Statusword of the Profile Velocity Mode ..................................................................39
7.2.5 Operation in the Profile Velocity Mode .....................................................................40
7.3 Profile Position Mode (pp) ...............................................................................................41
7.3.1 General Information .........................................................................................................41
7.3.2 Related Objects..................................................................................................................42
7.3.3 Controlword of the Profile Position Mode ...............................................................43
7.3.4 Statusword of the Profile Position Mode ..................................................................44
7.3.5 Operation in the Profile Position Mode ....................................................................45
7.3.6 Positioning option code (60F2h) .................................................................................46
7.4 Profile Torque Mode (tq) ...................................................................................................47
7.4.1 General Information .........................................................................................................47
7.4.2 Related Objects..................................................................................................................47
7.4.3 Controlword of the Profile Torque Mode ..................................................................48
7.4.4 Statusword of the Profile Torque Mode ....................................................................49
7.4.5 Operation in the Profile Torque Mode .......................................................................50
7.5 Homing Mode (hm) ............................................................................................................51
7.5.1 General Information .........................................................................................................51
7.5.2 Related Objects..................................................................................................................51
7.5.3 Controlword of the Homing Mode .............................................................................52
7.5.4 Statusword of the Homing Mode ...............................................................................53
7.5.5 Homing method ................................................................................................................54
7.6 Touch probe functionality ...............................................................................................58
7.6.1 General Information .........................................................................................................58
7.6.2 Related Objects..................................................................................................................58
7.6.3 Example of Execution Procedure for a Touch Probe ............................................59
8 Object Dictionary ...............................................................................................60
8.1 Communication Objects ..................................................................................................60
1000h: Device Type .........................................................................................................................60
1001h: Error register .......................................................................................................................60
1003h: Pre-defined error field .....................................................................................................61
1005h: COB-ID SYNC message ....................................................................................................62
1006h: Communication cycle period .......................................................................................63
1008h: Manufacturer device name ...........................................................................................63
1009h: Manufacturer hardware version ..................................................................................63
100Ah: Manufacturer software version ...................................................................................63
100Ch: Guard time ..........................................................................................................................63
100Dh: Life time factor ..................................................................................................................63
1010h: Store parameters ...............................................................................................................64
1011h: Restore default parameters ...........................................................................................65
3

## Page 4

Table of contents
1014h: COB-ID EMCY ......................................................................................................................66
1016h: Consumer heartbeat time ..............................................................................................66
1017h: Producer heartbeat time ................................................................................................67
1018h: Identity object ....................................................................................................................67
1200h: SDO server parameter .....................................................................................................67
1400h: 1st RPDO communication parameter ........................................................................68
1401h: 2nd RPDO communication parameter ......................................................................69
1402h: 3rd RPDO communication parameter .......................................................................70
1403h: 4th RPDO communication parameter .......................................................................71
1600h: 1st RPDO mapping parameter .....................................................................................72
1601h: 2nd RPDO mapping parameter ...................................................................................73
1602h: 3rd RPDO mapping parameter.....................................................................................74
1603h: 4th RPDO mapping parameter ....................................................................................75
1800h: 1st TPDO communication parameter ........................................................................76
1801h: 2nd TPDO communication parameter ......................................................................78
1802h: 3rd TPDO communication parameter .......................................................................80
1803h: 4th TPDO communication parameter .......................................................................82
1A00h: 1st TPDO mapping parameter .....................................................................................84
1A01h: 2nd TPDO mapping parameter ...................................................................................85
1A02h: 3rd TPDO mapping parameter ....................................................................................86
1A03h: 4th TPDO mapping parameter ....................................................................................87
8.2 Manufacturer Specific Objects .......................................................................................88
402Ch: Direct data operation operation data number ......................................................88
402Dh: Direct data operation operation type .......................................................................88
402Eh: Direct data operation position .....................................................................................88
402Fh: Direct data operation operating velocity .................................................................88
4030h: Direct data operation acceleration rate ....................................................................88
4031h: Direct data operation deceleration rate ...................................................................88
4032h: Direct data operation torque limiting value ...........................................................88
4033h: Direct data operation trigger ........................................................................................89
4034h: Direct data operation forwarding destination .......................................................89
403Ah: Driver input command (2nd) ........................................................................................89
403Ch: Driver input command (automatic OFF) ..................................................................89
403Dh: NET selection data number ..........................................................................................89
403Eh: Driver input command ....................................................................................................89
403Fh: Driver output status .........................................................................................................89
404Bh: Target position (User-defined position unit) ...........................................................90
404Ch: Demand position (User-defined position unit) ......................................................90
404Dh: Actual position (User-defined position unit) ..........................................................90
404Eh: Target velocity (User-defined velocity unit) .............................................................90
404Fh: Demand velocity (User-defined velocity unit) ........................................................90
4050h: Actual velocity (User-defined velocity unit) ............................................................90
4056h: Present communication error .......................................................................................90
4

## Page 5

Table of contents
406Bh: Torque monitor ..................................................................................................................90
406Ch: Load factor monitor .........................................................................................................91
406Dh: Cumulative load monitor ..............................................................................................91
4070h: Next data number .............................................................................................................91
4071h: Loop origin data number ...............................................................................................91
4072h: Loop count ..........................................................................................................................91
4073h: Position deviation .............................................................................................................91
4075h: Speed deviation .................................................................................................................91
407Ah: Tripmeter 1 ..........................................................................................................................91
407Bh: Present information .........................................................................................................92
407Ch: Driver temperature ...........................................................................................................92
407Dh: Motor temperature ..........................................................................................................92
407Eh: Odometer .............................................................................................................................92
407Fh: Tripmeter 0 ...........................................................................................................................92
409Bh: Main power supply current ...........................................................................................92
409Ch: Power consumption .........................................................................................................92
409Dh: Energy consumption .......................................................................................................92
409Eh: User energy consumption..............................................................................................93
409Fh: Total energy consumption .............................................................................................93
40A1h: Total uptime ........................................................................................................................93
40A2h: Number of boots ...............................................................................................................93
40A3h: Inverter voltage .................................................................................................................93
40A4h: Main power supply voltage ..........................................................................................93
40A9h: Continuous uptime ..........................................................................................................93
40AAh: RS-485 communication reception byte counter ...................................................93
40ABh: RS-485 communication transmission byte counter .............................................94
40C0h: Alarm reset ..........................................................................................................................94
40C2h: Clear alarm history ...........................................................................................................94
40C5h: P-PRESET execution .........................................................................................................94
40C6h: Configuration .....................................................................................................................94
40CDh: Clear latch information ..................................................................................................94
40CEh: Clear sequence history ....................................................................................................94
40D0h: Clear ETO .............................................................................................................................94
40D1h: ZSG-PRESET ........................................................................................................................94
40D2h: Clear ZSG-PRESET .............................................................................................................95
40D3h: Clear information ..............................................................................................................95
40D6h: Clear user energy consumption ..................................................................................95
40D7h: Clear tripmeter 0 ...............................................................................................................95
40D8h: Clear tripmeter 1 ...............................................................................................................95
4148h: Permission of absolute positioning without setting absolute
coordinates .........................................................................................................................95
415Fh: JOG/HOME Torque limit value ......................................................................................95
4160h: (HOME) Homing mode ....................................................................................................95
5

## Page 6

Table of contents
4163h: (HOME) Starting velocity ................................................................................................95
4169h: (HOME) Backward steps in 2 sensor homeseeking ...............................................96
4186h: Stopping timeout at alarm generation .....................................................................96
41CAh: WRAP setting .....................................................................................................................96
4735h: Custom stopping rate ......................................................................................................96
4736h: Custom stopping time ....................................................................................................96
8.3 Device Profile Objects .......................................................................................................97
603Fh: Error code .............................................................................................................................97
6040h: Controlword ........................................................................................................................97
6041h: Statusword...........................................................................................................................98
605Ah: Quick stop option code ..................................................................................................99
605Bh: Shutdown option code ...................................................................................................99
605Ch: Disable operation option code ....................................................................................99
605Dh: Halt option code ...............................................................................................................99
605Eh: Fault reaction option code ..........................................................................................100
6060h: Modes of operation .......................................................................................................100
6061h: Modes of operation display ........................................................................................100
6062h: Position demand value ................................................................................................100
6064h: Position actual value .....................................................................................................100
6065h: Following error window ..............................................................................................100
6067h: Position window .............................................................................................................101
606Bh: Velocity demand value .................................................................................................101
606Ch: Velocity actual value .....................................................................................................101
606Dh: Velocity window .............................................................................................................101
606Fh: Velocity threshold ..........................................................................................................101
6071h: Target torque ...................................................................................................................101
6072h: Max torque .......................................................................................................................101
6074h: Torque demand ...............................................................................................................102
6077h: Torque actual value .......................................................................................................102
607Ah: Target position ................................................................................................................102
607Bh: Position range limit .......................................................................................................102
607Ch: Home offset .....................................................................................................................102
607Dh: Software position limit ...............................................................................................103
6081h: Profile velocity .................................................................................................................103
6082h: End velocity ......................................................................................................................103
6083h: Profile acceleration ........................................................................................................103
6084h: Profile deceleration .......................................................................................................104
6085h: Quick stop deceleration ...............................................................................................104
6087h: Torque slope .....................................................................................................................104
608Fh: Position encoder resolution .......................................................................................104
6091h: Gear ratio ...........................................................................................................................105
6098h: Homing method .............................................................................................................105
6099h: Homing speeds ...............................................................................................................105
6

## Page 7

Table of contents
609Ah: Homing acceleration ....................................................................................................106
60A8h: SI unit position ................................................................................................................106
60A9h: SI unit velocity .................................................................................................................107
60B8h: Touch probe function ...................................................................................................108
60B9h: Touch probe status ........................................................................................................109
60BAh: Touch probe 1 positive edge .....................................................................................109
60BBh: Touch probe 1 negative edge ....................................................................................109
60BCh: Touch probe 2 positive edge .....................................................................................109
60BDh: Touch probe 2 negative edge ...................................................................................109
60D5h: Touch probe 1 positive edge counter ....................................................................110
60D6h: Touch probe 1 negative edge counter ..................................................................110
60D7h: Touch probe 2 positive edge counter ....................................................................110
60D8h: Touch probe 2 negative edge counter ..................................................................110
60E3h: Supported homing methods .....................................................................................111
60F2h: Positioning option code ..............................................................................................112
60F4h: Following error actual value .......................................................................................113
60FDh: Digital inputs ...................................................................................................................113
60FEh: Digital outputs.................................................................................................................114
60FFh: Target velocity (pv) .........................................................................................................114
6502h: Supported drive modes ...............................................................................................115
67FEh: Version number ...............................................................................................................116
67FFh: Single device type ..........................................................................................................116
9 Appendix ............................................................................................................117
9.1 Object list ............................................................................................................................117
9.2 Specifications ....................................................................................................................128
7

## Page 8

General Information
1 General Information
1.1 About this manual
This manual describes the setup, range of functions and software protocol of the BLVD-KRD and BLVD-KBRD
brushless motor driver with the CANopen communication profile.
The installation and setup of the driver, as well as all standard functions, are described in the corresponding product
manuals.
Manual number
Title Publisher
Japanese English
BLV Series R Type OPERATING MANUAL
Orientalmotor HP-5139 HP-5140
Installation and Connection Edition
BLV Series R Type OPERATING MANUAL
Installation and Connection Edition Orientalmotor HP-5162 HP-5163
Driver: BLVD-KBRD
BLV Series R Type OPERATING MANUAL
Orientalmotor HP-5141 HP-5142
Function Edition
Operating manuals are not included with the product. Download them from Oriental Motor Website Download Page
or contact your nearest Oriental Motor sales office.
Additional documentation:
Title Publisher
CiA documents CiA.e.V.
ISO 11898: Controller Area Network (CAN) for high-speed communication –
This manual is intended for the following qualified personnel:
Wiring: Professionally qualified electrical technicians
Programming: Software developers, project-planners
8

## Page 9

General Information
1.2 Object description
Notation Description
Index 16-bit address to access information in the CANopen object dictionary.
Sub-index For array and records the address is extended by an 8-bit sub-index.
Data type The following table lists the data types and ranges that are used in this manual.
Code Data Type Range
INT8 Signed 8-bit integer (INTEGER8) –128 to 127
INT16 Signed 16-bit integer (INTEGER16) –32,768 to 32,767
INT32 Signed 32-bit integer (INTEGER32) –2,147,483,648 to 2,147,483,647
UINT8 Unsigned 8-bit integer (UNSIGNED8) 0 to 255
UINT16 Unsigned 16-bit integer (UNSIGNED16) 0 to 65,535
UINT32 Unsigned 32-bit integer (UNSIGNED32) 0 to 4,294,967,295
STRING Character string (Visible String) Same UINT8
Access Access method of objects.
• rw: Read and write of values are possible.
• ro: Only read of values is possible.
• rww: Read/write on process output
• c: Read only, value will not change
PDO Indicates whether the PDO mapping of objects is possible.
• Yes: Mapping to PDO is possible.
• No: Mapping to PDO is not possible.
Unit The following table lists the data units and notations that are used in this manual.
Notation Description
You can define the position unit.
Pos. unit
Default: step
You can define the velocity unit.
Vel. unit
Default: r/min
Velocity unit per second (Vel. unit / s)
Acc. unit
Default: (r/min) / s
Velocity unit per second (Vel. unit / s) or ms
Acc. unit (MS) Default: ms
According to “User-defined acceleration/deceleration unit setting” parameter.
Refer to the following for details on the units.
- OPERATING MANUAL BLV Series R Type Function Edition
Save Indicates whether data is saved in the non-volatile memory when the batch non-volatile memory
write was executed.
• Yes: Saved in the non-volatile memory.
• No : Not saved in the non-volatile memory.
9

## Page 10

Installation and Setup
2  Installation and Setup
Setup the driver as described in the operation manuals for BLV Series R Type. 
Observe all safety instructions in the installation instructions that belong to the driver.
Follow all the notes on mounting position, ambient conditions, wiring, and fusing.
Refer to the following for details on the installation and setup.
- BLV Series R Type OPERATING MANUAL Installation and Connection Edition
2.1  Installation and connection
Support software
Host controller
CN4
Power supply for
communication
* BLVD-KBRD does not 
   require a power supply 
     for communication.
Motor
|     | CN3 CN2            | CN1                 | Breaker |     |
| --- | ------------------ | ------------------- | ------- | --- |
|     | (upper row)        | (lower row)         |         |     |
|     | Connecting a motor | Connecting          |         |     |
|     |  and a driver      | a main power supply |         |     |
DC power
supply
| CN4   | Pin No. | Signal name |                                       | Description |
| ----- | ------- | ----------- | ------------------------------------- | ----------- |
| 1 7   | 14 5    | CAN_L       | CAN Low                               |             |
|       | 6       | CAN_H       | CAN High                              |             |
|       | 7       | CAN_GND     | Ground for CAN communication          |             |
|       | 20      | NET-VIN     | Power supply for communication        |             |
| 15 21 | 28 21   | NET-GND     | Ground for power supply communication |             |
10

## Page 11

Installation and Setup
2.2 Connection of termination resistor
CANopen
Driver 1 Driver n
master
CAN_L
R CAN-BUS cable R
CAN_H
CAN_GND
R: Termination resistor
Connect the termination resistor (120 Ω, 1/4 W or more) on both ends of a bus. Termination resistors are not included
with the product.
2.3 Setting the Bitrate and Node-ID
z Setting the Node-ID
The Node-ID can be set in two different ways:
- By using the support software
- By using the ID-SEL input
Possible Node-ID: 1 to 127
Refer to the following for details on the ID-SEL input.
- BLV Series R Type OPERATING MANUAL Function Edition
z Setting the Bitrate
The Node-ID can be set following way:
- By using the support software
Possible Bitrate: 1000, 800, 500 (default), 250, 125, 50, 20, and 10 kbps
Bus length
Bitrate [kbit/s] Maximum bus length [m]
1,000 25
800 50
500 (default) 100
250 250
125 500
50 1,000
20 2,500
10 5,000
11

## Page 12

Communication specifications
3 Communication specifications
3.1 CANopen device model
A CANopen device is structured like the following.
- Communication
This function unit provides the communication objects and the appropriate functionality to transport data items via
the underlying network structure.
- Object dictionary
The object dictionary is a collection of all the data items which have an influence on the behavior of the application
objects, the communication objects and the state machine used on this device.
- Application
The application comprises the functionality of the device with respect to the interaction with the process
environment.
Thus the object dictionary serves as an interface between the communication and the application.
A device profile is a description of a device’s application in terms of the individual data contained in the object
dictionary.
CANopen
Communication Application
State machine
Application object
Object
dictionary
Process data object
(PDO)
1000h
Service data object Application object
(SDO) :
Power supply
2000h / Motor
SYNC, EMCY
: Application object
Network 6000h
management (NMT)
:
Application object
FFFFh
NMT error control
CAN-BUS
12

## Page 13

Communication specifications
3.2 General description of CAN
The transmission method that is used here is defined in ISO 11898 (Controller Area Network CAN for high-speed
communication).
Physical Layer/Data Link Layer that is implemented in all CAN modules provides, amongst other things, the
requirements for data.
Data transport or data request is made by means of a data telegram (Data Frame) with up to 8 bytes of user data, or
by a data request telegram (Remote Frame).
Communication Objects are labeled by an 11 bit Identifier (COB-ID) that also determines the priority of Objects.
Application Layer was developed, to decouple the application from the communication.
The service elements that are provided by the Application Layer make it possible to implement an application that is
spread across the network. These service elements are described in the CAN Application Layer (CAL) for Industrial
Applications.
The communication profile CANopen and the drive profile are mounted on the CAL.
The basic structure of a Communication Object is shown in the following.
SOM COB-ID RTR CTRL DATA CRC ACK EOM
Notation Description
SOM Start of message (1 bit)
COB-ID Communication object identifier (11 bits)
RTR Remote transmission request (1 bit)
CTRL Control field (e.g. Data length code) (6 bit)
DATA Data field (0 to 8 bytes)
CRC Cyclic redundancy check (15 bits) and CRC delimiter (1 bit)
ACK Acknowledge slot (1 bit) and acknowledge delimiter (1 bit)
EOM End of message (7 bit)
13

## Page 14

Communication specifications
3.3  Communication Object Identifier (COB-ID)
The following diagram shows the layout of the communication object Identifier (COB-ID). 
The Function Code defines the interpretation and priority of the particular Object.
z  Data Description of COB-ID
| Bit 10 | 9             | 8 7 6 | 5 4 | 3 2     | 1   | 0   |     |
| ------ | ------------- | ----- | --- | ------- | --- | --- | --- |
|        | Function Code |       |     | Node-ID |     |     |     |
The priority is higher when the value is lower toward zero.
Thus the highest priority has the function code of zero. Higher priority message takes precedence on the CAN bus 
and the lower one will have to wait. The arbitration of the CAN bus is done at the CAN hardware level, thus ensuring 
that the highest priority message is transmitted first.
The following table shows the COB-ID on the driver.
| Communication  |     |             | Function  |         |     |                  | Related  |
| -------------- | --- | ----------- | --------- | ------- | --- | ---------------- | -------- |
|                |     | Description |           | Node-ID |     | Resulting COB-ID |          |
| object (COB)   |     |             | code      |         |     |                  | objects  |
Network Management 
| NMT |     |     | 0000b | 0000000b |     | 0 (000h) | –   |
| --- | --- | --- | ----- | -------- | --- | -------- | --- |
(broadcast)
|      | Synchronization message  |     |       |          |     |            | 1005h  |
| ---- | ------------------------ | --- | ----- | -------- | --- | ---------- | ------ |
| SYNC |                          |     | 0001b | 0000000b |     | 128 (080h) |        |
|      | (broadcast)              |     |       |          |     |            | 1006h  |
EMCY Emergency messages 0001b Node-ID 128 (080h) + Node-ID 1014h
TPDO1 Transmit Process Data Object 1 0011b Node-ID 384 (180h) + Node-ID 1800h
RPDO1 Receive Process Data Object 1 0100b Node-ID 512 (200h) + Node-ID 1400h
TPDO2 Transmit Process Data Object 2 0101b Node-ID 640 (280h) + Node-ID 1801h
RPDO2 Receive Process Data Object 2 0110b Node-ID 768 (300h) + Node-ID 1401h
TPDO3 Transmit Process Data Object 3 0111b Node-ID 896 (380h) + Node-ID 1802h
RPDO3 Receive Process Data Object 3 1000b Node-ID 1024 (400h) + Node-ID 1402h
TPDO4 Transmit Process Data Object 4 1001b Node-ID 1152 (480h) + Node-ID 1803h
RPDO4 Receive Process Data Object 4 1010b Node-ID 1280 (500h) + Node-ID 1403h
TSDO Transmit Service Data Objects 1011b Node-ID 1408 (580h) + Node-ID 1200h
RSDO Receive Service Data Objects 1100b Node-ID 1536 (600h) + Node-ID 1200h
100Ch 
| NMT error  | Network management error  |     |       |         |                       |     | 100Dh  |
| ---------- | ------------------------- | --- | ----- | ------- | --------------------- | --- | ------ |
|            |                           |     | 1110b | Node-ID | 1792 (700h) + Node-ID |     |        |
| control    | control                   |     |       |         |                       |     | 1016h  |
1017h
Note: NMT Error Control includes Node Guarding and Heartbeat.
z  Restricted COB-IDs
Any COB-ID listed in the following table is of restricted use. Such a restricted COB-ID is not used as a COB-ID by any 
configurable communication object, neither for SYNC, EMCY, PDO, and SDO.
|                            | COB-ID   |     | Used by COB       |     |     |     |     |
| -------------------------- | -------- | --- | ----------------- | --- | --- | --- | --- |
|                            | 0 (000h) |     | NMT               |     |     |     |     |
| 1 (001h) to 127 (07Fh)     |          |     | Reserved          |     |     |     |     |
| 257 (101h) to 384 (180h)   |          |     | Reserved          |     |     |     |     |
| 1409 (581h) to 1535 (5FFh) |          |     | Default TSDO      |     |     |     |     |
| 1537 (601h) to 1663 (67Fh) |          |     | Default RSDO      |     |     |     |     |
| 1760 (6E0h) to 1791 (6FFh) |          |     | Reserved          |     |     |     |     |
| 1793 (701h) to 1919 (77Fh) |          |     | NMT Error Control |     |     |     |     |
| 2020 (780h) to 2047 (7FFh) |          |     | Reserved          |     |     |     |     |
14

## Page 15

Communication specifications
3.4  Endian format
CANopen uses Little Endian format. For numerical data greater than 1 byte, the least significant byte (LSB) is stored in 
the lowest order of the memory. When sending data, the LSB is sent first.
Example:
A 32 bit number has the value of 0x00112233. The 4 bytes data when assembled into the data field would look like 
the following.
| Byte 0 | 1 2     | 3 4    | 5 6   | 7   |
| ------ | ------- | ------ | ----- | --- |
| 33h    | 22h 11h | 00h xx | xx xx | xx  |
15

## Page 16

Communication Objects
4  Communication Objects
4.1  Network Management (NMT)
The network management follows a master-slave structure. NMT requires a CANopen device in the network that 
performs the role of the NMT master. All other devices have the role of the NMT slave. 
Each NMT slave can be addressed via its individual Node-ID in the range from [1–127]. 
NMT services can be used to initiate, start, monitor, reset or stop CANopen devices.
In doing so, the driver follows the state diagram shown in the following. The “Initialization” state is only reached after 
power on or by sending a “Reset Communication” or “Reset Node” NMT command.
The “Pre-operational” state is automatically activated after initialization.
Power on
NMT Network management
SDO Service data object
1
SYNC Synchronizing object
Initialization
Emergency object
EMCY
PDO Process data object
2
| 14 Pre-operational |     | 11  |     |     |
| ------------------ | --- | --- | --- | --- |
 
| NMT SDO | SYNC EMCY |     |     |     |
| ------- | --------- | --- | --- | --- |
7
4
5
| 13  |     |     | Stopped | 10  |
| --- | --- | --- | ------- | --- |
NMT
6
3
| 12  |     | 8   |     | 9   |
| --- | --- | --- | --- | --- |
Operational
| NMT SDO                                                                          | SYNC EMCY | PDO |             |     |
| -------------------------------------------------------------------------------- | --------- | --- | ----------- | --- |
| State                                                                            |           |     | Description |     |
| 1 At power on the NMT state initialization is entered autonomously               |           |     |             |     |
| 2 NMT state initialization finish, enter NMT state Pre-operational automatically |           |     |             |     |
| 3 NMT service start remote node indication or by local control                   |           |     |             |     |
| 4, 7 NMT service enter Pre-operational indication                                |           |     |             |     |
| 5, 8 NMT service stop remote node indication                                     |           |     |             |     |
| 6 NMT service start remote node indication                                       |           |     |             |     |
| 9, 10, 11 NMT service reset node indication                                      |           |     |             |     |
| 12, 13, 14 NMT service reset communication indication                            |           |     |             |     |
Shown in the following table is an overview of the activity of the services in the respective states.
| Service | Initialization | Pre-operational | Operational | Stopped |
| ------- | -------------- | --------------- | ----------- | ------- |
| PDO     | –              | –               | Active      | –       |
| SDO     | –              | Active          | Active      | –       |
| SYNC    | –              | Active          | Active      | –       |
| EMCY    | –              | Active          | Active      | –       |
| NMT     | –              | Active          | Active      | Active  |
16

## Page 17

Communication Objects
NMT message:
| NMT master |     | NMT slaves |     |
| ---------- | --- | ---------- | --- |
Request
COB-ID DATA
 
Byte0 Byte1
0
|     | Command  | Indication |     |
| --- | -------- | ---------- | --- |
specifier Node-ID
DATA
Byte0 Byte1
| NMT message | COB-ID |     | Description |
| ----------- | ------ | --- | ----------- |
Command 
Node-ID
specifier
NMT service start remote node 0 01h Switch to the “Operational” state
NMT service stop remote node 0 02h Switch to the “Stoped” state
NMT service enter Pre-operational 0 80h Node-ID Switch to the “Pre-operational” state
| NMT service reset node          | 0 81h | Reset Node          |     |
| ------------------------------- | ----- | ------------------- | --- |
| NMT service reset communication | 0 82h | Reset Communication |     |
17

## Page 18

Communication Objects
4.2  NMT Error Control
The driver supports Node Guarding and Heartbeat protocol as NMT error controls.
4.2.1  Node Guarding
The NMT master polls each NMT slave at regular time intervals.
Set the time interval for node guarding message forwarding in “Guard time (100Ch)”.
The NMT master sends a message to the NMT slave at that cycle, and the NMT slave responds with node guarding 
message. The response of the NMT slave contains the NMT state of that NMT slave.
A remote node error is indicated through the NMT service node guarding event if
- The RTR is not confirmed within the guard time.
- The reported NMT slave state does not match the expected state.
The node life time is given by the guard time multiplied by the “Life time factor (100Dh)”. If the NMT slave has not 
been polled during its life time, a remote node error is indicated through the NMT service life guarding event.
If it has been indicated that a remote error has occurred and the errors in the guarding protocol have disappeared, it 
will be indicated that the remote error has been resolved through the NMT service node guarding event and the NMT 
service life guarding event.
z  Node guard request from NMT master
| COB-ID         | RTR |     |
| -------------- | --- | --- |
| 700h + Node-ID | 1   |     |
z  Node guard response from NMT slave
DATA
| COB-ID | RTR |     |
| ------ | --- | --- |
Byte0
| 700h + Node-ID | 0 toggle bit + NMT state |     |
| -------------- | ------------------------ | --- |
z  Data Description of Byte 0
| Bit | Name | Description |
| --- | ---- | ----------- |
4 (04h): Stopped 
| 0 to 6 | NMT state 5 (05h): Operational  |     |
| ------ | ------------------------------- | --- |
127 (7Fh): Pre-operational
| 7   | toggle bit This bit is toggled on each response (0 at start). |     |
| --- | ------------------------------------------------------------- | --- |
18

## Page 19

Communication Objects
| z  Node guarding message (in the case that NMT state is “Operational”) |     |     |     |     |
| ---------------------------------------------------------------------- | --- | --- | --- | --- |
NMT master NMT slaves
Request
|     |     | COB-ID | RTR |     |
| --- | --- | ------ | --- | --- |
Indication
700h + Node-ID
| emit draug edoN |     |     | 1   |     |
| --------------- | --- | --- | --- | --- |
Response
|     |     | COB-ID | RTR DATA |     |
| --- | --- | ------ | -------- | --- |
Confirmation
Byte0
|     |     | 700h + Node-ID | 0   |     |
| --- | --- | -------------- | --- | --- |
05h*
* toggle bit + NMT state
Request
|     |     | COB-ID | RTR | Node life time |
| --- | --- | ------ | --- | -------------- |
Indication
|     |     | 700h + Node-ID | 1   |     |
| --- | --- | -------------- | --- | --- |
Response
|     | Confirmation | COB-ID | RTR DATA |     |
| --- | ------------ | ------ | -------- | --- |
Byte0
|     |     | 700h + Node-ID | 0   |     |
| --- | --- | -------------- | --- | --- |
85h*
|     | Request |                | * toggle bit + NMT state |     |
| --- | ------- | -------------- | ------------------------ | --- |
|     |         | COB-ID         | RTR                      |     |
|     |         | 700h + Node-ID | 1                        |     |
No response
Indication
Node guarding event
19

## Page 20

Communication Objects
4.2.2 Heartbeat
The heartbeat protocol is an error control service without need for RTRs. A heartbeat producer transmits a heartbeat
message cyclically. A heartbeat message transmission cycle is defined by the “Producer heartbeat time (1017h)”. One
or more heartbeat consumer receives the indication. The relationship between producer and consumer is
configurable via the object dictionary. The heartbeat consumer monitors whether the heartbeat was received within
the time set in “Consumer heartbeat time (1016h)”.
If the heartbeat is not received within the consumer heartbeat time a heartbeat event will be generated.
If the heartbeat producer time is configured on the driver the heartbeat protocol begins immediately.
If the driver starts with a value for the Producer heartbeat time unequal to 0 the heartbeat protocol starts on the
transition from the NMT state Initialization to the NMT state Pre-operational.
In this case the boot-up message is regarded as first heartbeat message.
Notes:
- The consumer heartbeat time should be higher than the corresponding producer heartbeat time.
- A node is not allowed to support both Node Guarding and Heartbeat protocols at the same time.
z Heartbeat message
DATA
COB-ID
Byte0
700h + Node-ID NMT state
z Data Description of Byte 0
Bit Name Description
0 (00h): Boot-up
4 (04h): Stopped
0 to 6 NMT state
5 (05h): Operational
127 (7Fh): Pre-operational
7 reserved Always 0
z Heartbeat message (in the case that NMT state is “Operational”)
Heartbeat Heartbeat
producer consumers
Request
COB-ID DATA
Byte0
700h + Node-ID
05h
Request
COB-ID DATA
Byte0
700h + Node-ID
05h
Heartbeat event
20
emit
taebtraeh
recudorP
Consumer
heartbeat
time
Indication
Indication
No message

## Page 21

Communication Objects
4.3 NMT boot-up
The driver transmits a boot up message after power on, communication reset, or application reset events.
The protocol uses the same COB-ID as the error control protocols.
Boot-up message from CANopen slave
DATA
COB-ID
Byte0
700h + Node-ID 0
Boot-up Boot-up
producer producer
Request
COB-ID DATA
Byte0 Indication
700h + Node-ID
0
4.4 Synchronization object (SYNC)
The SYNC producer broadcasts the synchronization object periodically. This SYNC provides the basic network
synchronization mechanism. The time period between the SYNCs is specified by the standard parameter
“Communication cycle period (1006h)”. SYNC messages carry no data.
There must be only one SYNC producer in the network.
SYNC SYNC
producer consumer
Request
COB-ID
Indication
080h*
* Default value
The COB-ID SYNC message of the SYNC object is 80h by default. To enable Sync producer mode, bit 30 must be set in
“COB-ID SYNC message object (1005h)” after initialization of the network.
21

## Page 22

Communication Objects
| 4.5  | Emergency object (EMCY) |     |     |     |     |     |     |
| ---- | ----------------------- | --- | --- | --- | --- | --- | --- |
Emergency objects are triggered by the occurrence of the driver internal error situation and are transmitted from an 
emergency producer on the driver. Emergency objects are suitable for interrupt type error alerts.
An emergency object is transmitted only once per ‘error event’. No further emergency objects shall be transmitted as 
long as no new errors occur on the driver.
|     | z  Emergency message |     |     |     |     |     |     |
| --- | -------------------- | --- | --- | --- | --- | --- | --- |
DATA
COB-ID
|     |                | Byte 0 Byte 1    | Byte 2   | Byte 3 Byte 4 | Byte 5 | Byte 6 | Byte 7   |
| --- | -------------- | ---------------- | -------- | ------------- | ------ | ------ | -------- |
|     |                | Emergency error  | Error    |               |        |        |          |
|     | 080h + Node-ID |                  |          | 00h 00h       | 00h    | 00h    | 00h      |
|     |                | code             | register |               |        |        |          |
|     | EMCY           |                  |          |               |        |        | EMCY     |
|     | producer       |                  |          |               |        |        | consumer |
Request
|     |     | COB-ID  |                      | DATA           |            |     |            |
| --- | --- | ------- | -------------------- | -------------- | ---------- | --- | ---------- |
|     |     | 080h +  | Byte0                | Byte1 Byte2    | Byte3 to 7 |     | Indication |
|     |     | Node-ID | Emergency error code | Error register | 0          |     |            |
The driver is in one of two emergency states: Error Free or Error Occurred. Dependent on the transitions emergency 
objects is transmitted.
0.  After initialization the driver enters the error free state if no error is detected. No error message is sent.
1.  The driver detects an internal error indicated in the first three bytes of the emergency message (error code and 
error register). The driver enters the error state. An emergency object with the appropriate error code and error 
register is transmitted. The error code is filled in at the location of “Pre-defined error field (1003h)”. The value of the 
error register is also saved in “Error register object (1001h)”.
2.  One, but not all error reasons are gone. An emergency message containing error code 0000h (Error reset) may be 
transmitted together with the remaining errors in the error register.
3.  A new error occurs on the driver. The driver remains in error state and transmits an emergency object with the 
appropriate error code. The new error code is filled in at the top of the array of error codes (1003h). It is guaranteed 
that the error codes are sorted in a timely manner (oldest error - highest sub-index, see “Pre-defined error field 
(1003h)”).
4.  All errors are repaired. The driver enters the error free state and transmits an emergency object with the error code 
‘reset error / no error’.
5.  Reset or power-off.
start
0
error free
 
|     | 1   |     | 4   |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- |
2
5
error occurred
|     |     | 3   |     | end |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- |
22

## Page 23

Communication Objects
z  Error code and Error register
| Error code | Error register                                | Error description |
| ---------- | --------------------------------------------- | ----------------- |
| 0000h      | 00h Error Reset or no error                   |                   |
| 8110h      | 11h CAN overrun                               |                   |
| 8120h      | 11h CAN in error passive mode                 |                   |
| 8130h      | 11h Node guarding error or heartbeat error    |                   |
| 8140h      | 11h Recover from Bus-Off state                |                   |
| 8210h      | 11h PDO not processed due to length error     |                   |
| FF10h      | 81h Position deviation Possible               |                   |
| FF20h      | 81h Overcurrent Not possible Non-excitation   |                   |
| FF21h      | 81h Main circuit overheat                     |                   |
| FF22h      | 81h Overvoltage Non-excitation                |                   |
| FF25h      | 81h Undervoltage Non-excitation after         |                   |
| FF26h      | 81h Motor overheat deceleration               |                   |
| FF28h      | 81h Encoder error                             |                   |
| FF29h      | 81h Internal circuit error                    |                   |
| FF2Ah      | 81h Encoder communication error               |                   |
| FF30h      | 81h Overload                                  |                   |
| FF31h      | 81h Overspeed                                 |                   |
| FF41h      | 81h EEPROM error                              |                   |
| FF42h      | 81h Initial encoder error                     |                   |
| FF44h      | 81h Encoder EEPROM error                      |                   |
| FF45h      | 81h Motor combination error                   |                   |
| FF4Ah      | 81h Return-to-home incomplete                 |                   |
| FF50h      | 81h Electromagnetic brake overcurrent         |                   |
| FF53h      | 81h HWTO input circuit error                  |                   |
| FF55h      | 81h Electromagnetic brake connection error    |                   |
| FF60h      | 81h ±LS both sides active                     |                   |
| FF61h      | 81h Reverse ±LS connection                    |                   |
| FF62h      | 81h Return-to-home operation error            |                   |
| FF63h      | 81h No HOMES                                  |                   |
| FF64h      | 81h Z, SLIT signal error                      |                   |
| FF66h      | 81h Hardware overtravel                       |                   |
| FF67h      | 81h Software overtravel                       |                   |
| FF68h      | 81h HWTO input detection Non-excitation       |                   |
| FF6Ah      | 81h Return-to-home additional operation error |                   |
| FF70h      | 81h Operation data error                      |                   |
| FF71h      | 81h Unit setting error                        |                   |
| FF81h      | 81h Network bus error                         |                   |
| FF84h      | 81h RS-485 communication error                |                   |
| FF85h      | 81h RS-485 communication timeout              |                   |
| FF8Ch      | 81h Outside setting range                     |                   |
| FFF0h      | 81h CPU error                                 |                   |
| FFF3h      | 81h CPU overload                              |                   |
23

## Page 24

Communication Objects
4.6 Service data object (SDO)
A Service Data Object (SDO) is providing direct access to object entries of the driver’s object dictionary.
As these object entries may contain data of arbitrary size and data type.
SDO is a confirm service and operate under Client/Server model. SDOs may be used to transfer multiple data sets
(each containing an arbitrary large block of data) from a client to a server and vice versa.
The client can control via index and sub-index of the object dictionary which data set shall be transferred.
The content of the data set is defined within the object dictionary.
Basically an SDO is transferred as a sequence of segments. Prior to transferring the segments there is an initialization
phase where client and server prepare themselves for transferring the segments.
For SDOs, it is also possible to transfer a data set of up to 4 bytes during the initialization phase.
This mechanism is called SDO expedited transfer.
The following communication services are supported.
- SDO download initiate
- SDO download segment
- SDO upload initiate
- SDO upload segment
- SDO abort transfer
z SDO download
The client is using the service SDO download for transferring data from the client to the server (owner of the object
dictionary: the driver). The SDO download consists of at least the SDO download initiate service and optionally of the
SDO download segment services (data length > 4 bytes).
SDO download (normal) SDO download (expedited)
Client Server Client Server
SDO download initiate SDO download initiate
(e = 0) (e = 1)
SDO download segment
(t = 0, c = 0)
e : 0: normal
1: expedited
SDO download segment
(t = 1, c = 0) t : toggle bit
c : 0: more segments to be download
1: no more segments to be download
SDO download segment
(t = 0, c = 0)
:
:
SDO download segment
(t = x, c = 1)
24

## Page 25

Communication Objects
z SDO upload
The client is using the service SDO upload for transferring the data from the server (owner of the object dictionary:
driver) to the client. The SDO upload consists of at least the SDO upload initiate service and optional of SDO upload
segment services (data length > 4 bytes).
SDO upload (normal) SDO upload (expedited)
Client Server Client Server
SDO upload initiate SDO upload initiate
(e = 0) (e = 1)
SDO upload segment
(t = 0, c = 0)
e : 0: normal
1: expedited
SDO upload segment
(t = 1, c = 0) t : toggle bit
c : 0: more segments to be upload
1: no more segments to be upload
SDO upload segment
(t = 0, c = 0)
:
:
SDO upload segment
(t = x, c = 1)
25

## Page 26

Communication Objects
4.6.1  SDO download initiate
The client requests the server to prepare downloading of data by using the SDO download initiate service. 
The multiplexer of the data set and the transfer type are indicated to the server. 
In case of an SDO expedited download, the data of the data set identified by the Index and Sub-index is indicated to 
the server.
z  SDO download initiate message
Client request
DATA
| COB-ID  |                   | Byte 0      |                   |               |               |               |        |
| ------- | ----------------- | ----------- | ----------------- | ------------- | ------------- | ------------- | ------ |
|         |                   |             |                   | Byte 1 Byte 2 | Byte 3 Byte 4 | Byte 5 Byte 6 | Byte 7 |
|         | Bit 7 Bit 6 Bit 5 | Bit 4 Bit 3 | Bit 2 Bit 1 Bit 0 |               |               |               |        |
| 600h +  |                   |             |                   | Index  Index  | Sub-          |               |        |
|         | CCS=1             | x           | n e               | s             |               | d             |        |
| Node-ID |                   |             |                   | (LSB) (MSB)   | Index         |               |        |
Server response
DATA
| COB-ID  |                   | Byte 0      |                   |               |               |                   |        |
| ------- | ----------------- | ----------- | ----------------- | ------------- | ------------- | ----------------- | ------ |
|         |                   |             |                   | Byte 1 Byte 2 | Byte 3 Byte 4 | Byte 5 Byte 6     | Byte 7 |
|         | Bit 7 Bit 6 Bit 5 | Bit 4 Bit 3 | Bit 2 Bit 1 Bit 0 |               |               |                   |        |
| 580h +  |                   |             |                   | Index  Index  | Sub-          |                   |        |
|         | SCS=3             |             | x                 |               |               | Reserved always 0 |        |
| Node-ID |                   |             |                   | (LSB) (MSB)   | Index         |                   |        |
- CCS: client command specifier
  1: initiate download request
- SCS: server command specifier
  3: initiate download response
- n: Only valid if e = 1 and s = 1, otherwise 0. 
  If valid it indicates the number of bytes in d that do not contain data. Bytes [8-n, 7] do not contain data.
- e: transfer type
  0: normal transfer
  1: expedited transfer
- s: size indicator
  0: data set size is not indicated
  1: data set size is indicated
- d: data
  e = 0, s = 0: d is reserved for further use.
  e = 0, s = 1: d contains the number of bytes to be downloaded.
|     | Byte 4 contains the LSB and byte 7 contains the MSB. |     |     |     |     |     |     |
| --- | ---------------------------------------------------- | --- | --- | --- | --- | --- | --- |
 e = 1, s = 1: d contains the data of length 4-n to be downloaded. The encoding depends on the type of the data 
referenced by Index and Sub-index.
  e = 1, s = 0: d contains unspecified number of bytes to be downloaded.
- x: not used, always 0
26

## Page 27

Communication Objects
4.6.2 SDO download segment
The client transfers the segmented data to the server by using the SDO download service. The segment data and
optionally its size are indicated to the server. The continue parameter indicates the server whether there are still more
segments to be downloaded or that this was the last segment to be downloaded.
z SDO download segment message
Client request
DATA
COB-ID Byte 0
Byte 1 Byte 2 Byte 3 Byte 4 Byte 5 Byte 6 Byte 7
Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
600h +
CCS=0 t n c seg-data
Node-ID
Server response
DATA
COB-ID Byte 0
Byte 1 Byte 2 Byte 3 Byte 4 Byte 5 Byte 6 Byte 7
Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
580h +
SCS=1 t x Reserved always 0
Node-ID
- CCS: client command specifier
0: download segment request
- SCS: server command specifier
1: download segment response
- seg-data: segments data
At most 7 bytes of segment data to be downloaded. The encoding depends on the type of the data
referenced by index and sub-index.
- n: number of bytes
Indicates the number of bytes in segment data that do not contain segment data.
Bytes [8-n, 7] do not contain segment data. If n = 0 bytes 1 to 7 shall contain segment data.
NOTE: If the size in the initiation is indicated this applies to the overall data transferred.
- c: more segments
Indicates whether there are still more segments to be downloaded.
0: more segments to be downloaded
1: no more segments to be downloaded
- t: toggle bit
This bit is alternate for each subsequent segment that is downloaded. The first segment has the toggle-bit set to 0.
The toggle bit is equal for the request and the response message.
- x: not used, always 0
27

## Page 28

Communication Objects
4.6.3  SDO upload initiate
The client requests the server to prepare the data for uploading by using the SDO upload initiate service. The Index 
and Sub-index of the data set whose upload is initiated is indicated to the server.
z  SDO upload initiate message
Client request
DATA
| COB-ID  |                   | Byte 0      |                   |               |               |                   |        |
| ------- | ----------------- | ----------- | ----------------- | ------------- | ------------- | ----------------- | ------ |
|         |                   |             |                   | Byte 1 Byte 2 | Byte 3 Byte 4 | Byte 5 Byte 6     | Byte 7 |
|         | Bit 7 Bit 6 Bit 5 | Bit 4 Bit 3 | Bit 2 Bit 1 Bit 0 |               |               |                   |        |
| 600h +  |                   |             |                   | Index  Index  | Sub-          |                   |        |
|         | CCS=2             |             | x                 |               |               | Reserved always 0 |        |
| Node-ID |                   |             |                   | (LSB) (MSB)   | Index         |                   |        |
Server response
DATA
| COB-ID  |                   | Byte 0      |                   |               |               |               |        |
| ------- | ----------------- | ----------- | ----------------- | ------------- | ------------- | ------------- | ------ |
|         |                   |             |                   | Byte 1 Byte 2 | Byte 3 Byte 4 | Byte 5 Byte 6 | Byte 7 |
|         | Bit 7 Bit 6 Bit 5 | Bit 4 Bit 3 | Bit 2 Bit 1 Bit 0 |               |               |               |        |
| 580h +  |                   |             |                   | Index  Index  | Sub-          |               |        |
|         | SCS=2             | x           | n e               | s             |               | d             |        |
| Node-ID |                   |             |                   | (LSB) (MSB)   | Index         |               |        |
- CCS: client command specifier
  2: initiate upload request
- SCS: server command specifier
  2: initiate upload response
- n: number of bytes
  Only valid if e = 1 and s = 1, otherwise 0. If valid it indicates the number of bytes in d that do not contain data. 
Bytes [8-n, 7] do not contain segment data.
- e: transfer type
  0: normal transfer
  1: expedited transfer
- s: size indicator
  0: data set size is not indicated
  1: data set size is indicated
- d: data
  e = 0, s = 0: d is reserved for further use.
  e = 0, s = 1: d contains the number of bytes to be uploaded. 
|     | Byte 4 contains the LSB and byte 7 contains the MSB. |     |     |     |     |     |     |
| --- | ---------------------------------------------------- | --- | --- | --- | --- | --- | --- |
  e = 1, s = 1: d contains the data of length 4-n to be uploaded. The encoding depends on the type of the data 
referenced by Index and Sub-index.
  e = 1, s = 0: d contains unspecified number of bytes to be uploaded.
- x: not used, always 0
28

## Page 29

Communication Objects
4.6.4 SDO upload segment
The client requests the server to supply the data of the next segment by using the SDO upload segment service. The
continue parameter indicates the client whether there are still more segments to be uploaded or that this was the last
segment to be uploaded. There is at most one SDO upload segment service outstanding for an SDO.
z SDO upload segment message
Client request
DATA
COB-ID Byte 0
Byte 1 Byte 2 Byte 3 Byte 4 Byte 5 Byte 6 Byte 7
Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
600h +
CCS=3 t x Reserved always 0
Node-ID
Server response
DATA
COB-ID Byte 0
Byte 1 Byte 2 Byte 3 Byte 4 Byte 5 Byte 6 Byte 7
Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
580h +
SCS=0 t n c seg-data
Node-ID
- CCS: client command specifier
3: upload segment request
- SCS: server command specifier
0: upload segment response
- t: toggle bit
This bit shall alternate for each subsequent segment that is uploaded. The first segment shall have the toggle-bit
set to 0. The toggle bit shall be equal for the request and the response message.
- c: more segments
Indicates whether there are still more segments to be uploaded.
0: more segments to be uploaded
1: no more segments to be uploaded
- seg-data: segments data
At most 7 bytes of segment data to be uploaded. The encoding depends on the type of the data referenced
by Index and Sub-index.
- n: number of bytes
Indicates the number of bytes in seg-data that do not contain segment data.
Bytes [8-n, 7] do not contain segment data. If n = 0 bytes 1 to 7 shall contain segment data.
NOTE: If the size in the initiation is indicated this applies to the overall data transferred.
- x: not used, always 0
29

## Page 30

Communication Objects
4.6.5  SDO abort transfer
The SDO abort transfer service aborts the SDO upload service or SDO download service of an SDO referenced by its 
number. The reason is indicated. The service is unconfirmed. Both the client and the server of an SDO may execute the 
service at any time. If the client of an SDO has a confirmed service outstanding, the indication of the abort is taken to 
be the confirmation of that service.
z  SDO abort message
DATA
COB-ID Byte 0
|         |                         |                   | Byte 1 Byte 2 | Byte 3 Byte 4 | Byte 5 Byte 6 | Byte 7 |
| ------- | ----------------------- | ----------------- | ------------- | ------------- | ------------- | ------ |
| Bit 7   | Bit 6 Bit 5 Bit 4 Bit 3 | Bit 2 Bit 1 Bit 0 |               |               |               |        |
| 580h +  |                         |                   | Index  Index  | Sub-          |               |        |
|         | CS=4                    | x                 |               |               | d (LSB–MSB)   |        |
| Node-ID |                         |                   | (LSB) (MSB)   | Index         |               |        |
- CS: command specifier
  4: abort transfer request
- x: not used, always 0
- d: contains a 4 byte abort code about the reason for the abort.
z  SDO Abort Code
| Abort Code |                                                |     | Description |     |     |     |
| ---------- | ---------------------------------------------- | --- | ----------- | --- | --- | --- |
| 0503 0000h | Toggle bit not alternated                      |     |             |     |     |     |
| 0504 0001h | Command specifier not valid                    |     |             |     |     |     |
| 0601 0001h | Attempt to read a write only object            |     |             |     |     |     |
| 0601 0002h | Attempt to write a read only object            |     |             |     |     |     |
| 0602 0000h | Object does not exist in the object dictionary |     |             |     |     |     |
| 0604 0041h | Object cannot be mapped to the PDO             |     |             |     |     |     |
0604 0042h The number and length of the objects to be mapped would exceed PDO length
| 0606 0000h | Access failed due to a hardware error                                |     |     |     |     |     |
| ---------- | -------------------------------------------------------------------- | --- | --- | --- | --- | --- |
| 0607 0010h | Data type does not match, length of service parameter does not match |     |     |     |     |     |
| 0607 0012h | Data type does not match, length of service parameter too high       |     |     |     |     |     |
| 0607 0013h | Data type does not match, length of service parameter too low        |     |     |     |     |     |
| 0609 0011h | Sub-index does not exist                                             |     |     |     |     |     |
| 0609 0030h | Value range of parameter exceeded (only for write access)            |     |     |     |     |     |
| 0609 0031h | Value of parameter written too high                                  |     |     |     |     |     |
| 0609 0032h | Value of parameter written too low                                   |     |     |     |     |     |
| 0800 0000h | General error                                                        |     |     |     |     |     |
| 0800 0020h | Data cannot be transferred or stored to the application              |     |     |     |     |     |
0800 0022h Data cannot be transferred or stored to the application because of present device state
| 0800 0024h | No data available |     |     |     |     |     |
| ---------- | ----------------- | --- | --- | --- | --- | --- |
30

## Page 31

Communication Objects
4.7 Process data object (PDO)
The real-time data transfer is performed by means of “Process Data Objects (PDO)”. The transfer of PDO is performed
with no protocol overhead and it can be mapped to transport up to 8 data bytes in one CAN-frame.
The PDO correspond to objects in the object dictionary and provide the interface to the application objects.
Data type and mapping of application objects into a PDO is determined by a corresponding default PDO mapping
structure within the object dictionary.
There are two kinds of use for PDO. The first is data transmission and the second data reception.
It is distinguished in Transmit-PDO (TPDO) and Receive-PDO (RPDO). CANopen devices supporting TPDO are PDO
producer and CANopen devices supporting RPDO are called PDO consumer.
PDO are described by the PDO communication parameter and the PDO mapping parameter.
The driver supports the following parameters:
Receive-PDO (RPDO) (Master -> Driver)
- 1400h: 1st RPDO communication parameter
- 1401h: 2nd RPDO communication parameter
- 1402h: 3rd RPDO communication parameter
- 1403h: 4th RPDO communication parameter
- 1600h: 1st RPDO mapping parameter
- 1601h: 2nd RPDO mapping parameter
- 1602h: 3rd RPDO mapping parameter
- 1603h: 4th RPDO mapping parameter
Transmit-PDO (TPDO) (Driver -> Master)
- 1800h: 1st TPDO communication parameter
- 1801h: 2nd TPDO communication parameter
- 1802h: 3rd TPDO communication parameter
- 1803h: 4th TPDO communication parameter
- 1A00h: 1st TPDO mapping parameter
- 1A01h: 2nd TPDO mapping parameter
- 1A02h: 3rd TPDO mapping parameter
- 1A03h: 4th TPDO mapping parameter
Note: PDO can only be used if the NMT state machine is in the “Operational” state.
The PDO must be configured in the “Pre-operational” state.
z RPDO message
DATA
Name COB-ID
Byte 0 Byte 1 Byte 2 Byte 3 Byte 4 Byte 5 Byte 6 Byte 7
1st RPDO 200h + Node-ID 1st RPDO mapping parameter(1600h) Sub-index 01h to 04h
2nd RPDO 300h + Node-ID 2nd RPDO mapping parameter(1601h) Sub-index 01h to 04h
3rd RPDO 400h + Node-ID 3rd RPDO mapping parameter(1602h) Sub-index 01h to 04h
4th RPDO 500h + Node-ID 4th RPDO mapping parameter(1603h) Sub-index 01h to 04h
z TPDO message
DATA
Name COB-ID
Byte 0 Byte 1 Byte 2 Byte 3 Byte 4 Byte 5 Byte 6 Byte 7
1st TPDO 180h + Node-ID 1st TPDO mapping parameter(1A00h) Sub-index 01h to 04h
2nd TPDO 280h + Node-ID 2nd TPDO mapping parameter(1A01h) Sub-index 01h to 04h
3rd TPDO 380h + Node-ID 3rd TPDO mapping parameter(1A02h) Sub-index 01h to 04h
4th TPDO 480h + Node-ID 4th TPDO mapping parameter(1A03h) Sub-index 01h to 04h
31

## Page 32

Communication Objects
4.7.1 PDO mapping
You use the “RPDO mapping parameter (1600h to 1603h)” and the “TPDO mapping parameter (1A00h to 1A03h)”
objects to change the PDO mapping.
You change the PDO mapping as follows:
1. Deactivate the PDO by setting the Valid Bit (bit 31) of Sub-index 01h of the corresponding communication
parameter (e.g., 1400h: 01h) to “1”.
2. Deactivate the mapping by setting Sub-index 00h of the corresponding mapping parameter
(e.g., 1600h: 00h) to “0”.
3. Change the mapping in the desired Sub-index (e.g., 1600h: 01h).
4. Activate the mapping by writing the number of objects that are to be mapped in Sub-index 00h of the
corresponding mapping parameter (e.g., 1600h: 00h).
5. Activate the PDO by setting bit 31 of Sub-index 01h of the corresponding communication parameter
(e.g., 1400h: 01h) to “0”.
4.7.2 Transmission type
You can be set the transmission type as in the table below.
z RPDO transmission type
Transmission type PDO Transmission
00h Reflect the received data when receiving SYNC.
FEh Reflect the received data Immediately.*1*2
FFh Reflect the received data Immediately.
*1 The driver executes the following contents when received data in this transmission type.
- The driver Issue RTR to the corresponding TPDO
- The driver reset the node lifetime
*2 The driver with firmware version 2.02 or later support it.
(This "firmware version" does not mean "software version object (100Ah)")
z TPDO transmission type
Transmission type PDO Transmission
Sample it if the value of the mapping object has changed from the last transmission value.
00h
After that, transmit the value when receiving SYNC.
01h to F0h Sample and transmit the value of the mapping object at the “n” th SYNC reception.
F1h to FBh Reserved
Sample the value of the mapping object at the SYNC reception. After that, transmit the value
FCh
when receiving RTR.
FDh Sample and transmit the value of the mapping object at the RTR reception.
Sample and transmit the value of the mapping object when the following event.
- If the value of the mapping object is changed from the last transmission after the set time
FEh, FFh
of “Inhibit time (180xh: 03h)” has passed since the last transmission.
- If an event occurred by the “Event timer (180xh: 05h)”.
Note: An attempt to change the value of the transmission type to any not supported value is responded with the SDO
abort transfer service (abort code: 0609 0030h).
32

## Page 33

Device control
5  Device control
The device control of the driver can be used to carry out all the motion functions in the corresponding modes. The 
control of the driver is implemented through a mode-dependent status machine. The status machine is controlled 
through the Object “Controlword (6040h)”.
The states of the status machine can be revealed by using the Object “Statusword (6041h)”.
5.1  Status Machine
Start
|     |     |     |     | Main Power | NMT | Motor |
| --- | --- | --- | --- | ---------- | --- | ----- |
0
Not ready to
switch on
1
15
Switch on
|     |     |     | Fault | ON  | –   | Non-excitation |
| --- | --- | --- | ----- | --- | --- | -------------- |
disabled
2 7
Ready to
|     | 12 10 switch on |     |     |     |     |     |
| --- | --------------- | --- | --- | --- | --- | --- |
14
3 6
|     |     | 8 9 |     | ON  | Operational | Non-excitation |
| --- | --- | --- | --- | --- | ----------- | -------------- |
Switched on
Fault reaction
4 5
|            | 16        |     | active       |     |              |                   |
| ---------- | --------- | --- | ------------ | --- | ------------ | ----------------- |
| Quick stop | Operation |     |              |     |              |                   |
|            |           |     | 13           | ON  | Operational  | Excitation        |
| active     | enabled   |     |              |     |              |                   |
|            | 11        |     | Error occurs |     |              |                   |
|            | State     |     | Description  |     | Motor status | Parameter setting |
The main power supply was turned on, and 
| Not ready to switch on |     |     |     |     | Non-excitation | Not possible |
| ---------------------- | --- | --- | --- | --- | -------------- | ------------ |
the initialization is executing.
Switch on disabled The initialization was complete. Non-excitation Possible
Ready to switch on Drive functions cannot be carried out yet. Non-excitation Possible
Switched on Drive functions cannot be carried out yet. Non-excitation Possible
Operation enabled Drive functions are enabled. Excitation Possible
The Quick stop command was received, 
| Quick stop active |     |     |     |     | Excitation | Possible |
| ----------------- | --- | --- | --- | --- | ---------- | -------- |
and the operation stop is processing.
- A fault has occurred in the driver.
- A non-excitation alarm (or non-excitation 
| Fault reaction active |     |     |     |     | Excitation | Possible |
| --------------------- | --- | --- | --- | --- | ---------- | -------- |
after slow down alarm) has occurred in 
the driver, and the operation stop is 
processing.
- A non-excitation alarm (or non-excitation 
Fault after slow down alarm) is present in the  Non-excitation Possible
driver.
33

## Page 34

Status Machine control commands
6  Status Machine control commands
Bits in Controlword (6040h)
|                              |         |             | Enable    |            | Enable  | Switched  |              |
| ---------------------------- | ------- | ----------- | --------- | ---------- | ------- | --------- | ------------ |
|                              | Command | Fault reset |           | Quick stop |         |           | Transitions  |
|                              |         |             | operation |            | voltage | on        |              |
|                              |         | Bit7        | Bit3      | Bit2       | Bit1    | Bit0      |              |
| Shutdown                     |         | 0           | X         | 1          | 1       | 0         | 2, 6, 8      |
| Switch ON                    |         | 0           | 0         | 1          | 1       | 1         | 3            |
| Switch ON + Enable Operation |         | 0           | 1         | 1          | 1       | 1         | 3 + 4 *      |
| Disable Voltage              |         | 0           | X         | X          | 0       | X         | 7, 9, 10, 12 |
| Quick Stop                   |         | 0           | X         | 0          | 1       | X         | 7, 10, 11    |
| Disable Operation            |         | 0           | 0         | 1          | 1       | 1         | 5            |
| Enable Operation             |         | 0           | 1         | 1          | 1       | 1         | 4, 16        |
| Fault Reset                  |         | 0  1       | X         | X          | X       | X         | 15           |
Bits marked by an X are irrelevant.
* Automatic transition to enable operation state after executing switched on state functionality.
6.1  Bits in Statusword (6041h)
| Bit | Data Description |     |     |     | Remarks |     |     |
| --- | ---------------- | --- | --- | --- | ------- | --- | --- |
0 Ready to Switch ON
1 Switched ON
2 Operation Enabled
3 Fault
4 Voltage Enabled
5 Quick Stop
6 Switch ON Disabled
Refer to the following section for details.
| 7   | Warning |     | ->  7.2.4 Statusword of the Profile Velocity Mode  |     |     |     |     |
| --- | ------- | --- | -------------------------------------------------- | --- | --- | --- | --- |
Drive profile operation ready    7.3.4 Statusword of the Profile Position Mode 
8
(manufacturer-specific (MS))   7.4.4 Statusword of the Profile Torque Mode 
  7.5.4 Statusword of the Homing Mode
9 Remote
10 Target Reached
11 Internal Limit Active
12
Operation Mode Specific (OMS)
13
14 Reserved (manufacturer-specific (MS))
15 TLC (manufacturer-specific (MS))
34

## Page 35

Status Machine control commands
6.2  Transitions of the status machine
| Transition  |                 | Event  |     |                | Action |     |
| ----------- | --------------- | ------ | --- | -------------- | ------ | --- |
| 0           | Power on reset  |        |     | Initialization |        |     |
Activate communication and process data 
| 1   | Initialization completed successfully.  |     |     |     |     |     |
| --- | --------------------------------------- | --- | --- | --- | --- | --- |
monitoring.
| 2   | “Shutdown” command received from controlword.  |     |     | None |     |     |
| --- | ---------------------------------------------- | --- | --- | ---- | --- | --- |
| 3   | “Switch On” command received from controlword. |     |     | None |     |     |
“Enable Operation” command received from 
| 4   |     |     |     | The drive function is enabled. |     |     |
| --- | --- | --- | --- | ------------------------------ | --- | --- |
controlword.
“Disable operation” command received from 
| 5   |     |     |     | The drive function is disabled. |     |     |
| --- | --- | --- | --- | ------------------------------- | --- | --- |
controlword.
- “Shutdown” command received from controlword.
| 6   |     |     |     | None |     |     |
| --- | --- | --- | --- | ---- | --- | --- |
- “FREE signal input” is active.
- “Quick Stop” command received from controlword.
| 7   | - “QSTOP signal input” is active. |     |     | None |     |     |
| --- | --------------------------------- | --- | --- | ---- | --- | --- |
- “HWTO signal input” is active.
- “Shutdown” command received from controlword.
Drive function is disabled and the motor is 
8
|     | - “FREE signal input” is active. |     |     | free to rotate if unbraked. |     |     |
| --- | -------------------------------- | --- | --- | --------------------------- | --- | --- |
- “Disable Voltage” command received from 
controlword.
| 9   |     |     |     | Output stage is disabled. |     |     |
| --- | --- | --- | --- | ------------------------- | --- | --- |
- “HWTO signal input” is active.
- “Disable Voltage” or “Quick Stop” command received 
from controlword.
Drive function is disabled and the motor is 
10
|     | - “QSTOP signal input” is active. |     |     | free to rotate if unbraked. |     |     |
| --- | --------------------------------- | --- | --- | --------------------------- | --- | --- |
- “HWTO signal input” is active.
- “Quick Stop” command received from controlword.
| 11  |     |     |     | The Quick Stop function is executed. |     |     |
| --- | --- | --- | --- | ------------------------------------ | --- | --- |
- “QSTOP signal input” is active.
- “Quick Stop” function is completed or “Disable 
12 Voltage” command received from controlword. Drive function is disabled.
- “HWTO signal input” is active.
13 A fatal fault has occurred in the driver. Execute appropriate fault reaction.
14 The fault reaction is completed. The drive function is disabled.
- “Fault reset” command received from controlword.
| 15  |     |     |     | None |     |     |
| --- | --- | --- | --- | ---- | --- | --- |
- “ALM-RST signal input” is active.
| 16  | Not supported. |     |     |     | –   |     |
| --- | -------------- | --- | --- | --- | --- | --- |
6.3  Related Objects
| Index | Sub-index                         | Name | Type   | Access PDO mapping | Unit (default) |     |
| ----- | --------------------------------- | ---- | ------ | ------------------ | -------------- | --- |
| 6040h | 00h Controlword                   |      | UINT16 | rww                | Yes            | –   |
| 6041h | 00h Statusword                    |      | UINT16 | ro                 | Yes            | –   |
| 605Ah | 00h Quick stop option code        |      | INT16  | rw                 | No             | –   |
| 605Bh | 00h Shutdown option code          |      | INT16  | rw                 | No             | –   |
| 605Ch | 00h Disable operation option code |      | INT16  | rw                 | No             | –   |
| 605Dh | 00h Halt option code              |      | INT16  | rw                 | No             | –   |
| 605Eh | 00h Fault reaction option code    |      | INT16  | rw                 | No             | –   |
35

## Page 36

Operation mode
7  Operation mode
7.1  Modes of Operation
7.1.1  General Information
The driver supports the operation modes listed below.
- Profile Position Mode (pp)
- Profile Velocity Mode (pv)
- Profile Torque Mode (tq)*
- Homing Mode (hm)
* It is effective for the driver version 4.00 or later.
7.1.2  Related Objects
| Index | Sub-index                      | Name | Type   | Access PDO mapping |     | Unit (default) |
| ----- | ------------------------------ | ---- | ------ | ------------------ | --- | -------------- |
| 6060h | 00h Modes of operation         |      | INT8   | rww                | Yes | –              |
| 6061h | 00h Modes of operation display |      | INT8   | ro                 | Yes | –              |
| 6502h | 00h Supported drive modes      |      | UINT32 | ro                 | Yes | –              |
36

## Page 37

Operation mode
7.2 Profile Velocity Mode (pv)
7.2.1 General Information
In the Profile Velocity Mode, the speed is output according to the profile acceleration and profile deceleration until it
reaches the target velocity.
The following figure shows the block diagram for the Profile Velocity Mode.
z PV use a trajectory generator for positioning [When the (6040h: bit12) is 0]
Max torque (6072h)
Target velocity (60FFh)
Vel. unit
Profile acceleration (6083h)
Profile deceleration (6084h)
Trajectory Speed Torque
Quick stop deceleration (6085h) Generator control control
Motor
Quick stop option code (605Ah)
Halt option code (605Dh)
Torque actual value (6077h) Encoder
Velocity actual value (606Ch)
Vel. unit
Velocity window (606Dh)
Target reached in statusword Velocity
reached window
comparator
Velocity threshold (606Fh)
Speed in statusword Velocity
threshold
comparator
z PV use a trajectory generator for positioning [When the (6040h: bit12) is 1]
Max torque (6072h)
Target position (607Ah)
Position
Software position limit (607Dh) Limit function Pos. unit
Target velocity (60FFh)
Vel. unit
Profile acceleration (6083h)
Profile deceleration (6084h) Trajectory Position Speed Torque
Generator control control control
Quick stop deceleration (6085h) Motor
Quick stop option code (605Ah)
Halt option code (605Dh)
Encoder
Torque actual value (6077h)
Velocity actual value (606Ch)
Vel. unit
Velocity window (606Dh)
Target reached in statusword Velocity
reached window
comparator
Velocity threshold (606Fh)
Speed in statusword Velocity
threshold
comparator
Position actual value (6064h)
Pos. unit
37

## Page 38

Operation mode
7.2.2  Related Objects
| Index | Sub-index |     |                        | Name |     | Type Access | PDO mapping |     | Unit (default) |
| ----- | --------- | --- | ---------------------- | ---- | --- | ----------- | ----------- | --- | -------------- |
| 6040h | 00h       |     | Controlword            |      |     | UINT16      | rww         | Yes | –              |
| 6041h | 00h       |     | Statusword             |      |     | UINT16      | ro          | Yes | –              |
| 605Ah | 00h       |     | Quick stop option code |      |     | INT16       | rw          | No  | –              |
| 605Dh | 00h       |     | Halt option code       |      |     | INT16       | rw          | No  | –              |
6064h 00h Position actual value INT32 ro Yes Pos. unit (step)
606Ch 00h Velocity actual value INT32 ro Yes Vel. unit (r/min)
| 606Dh | 00h |     | Velocity window |     |     | UINT16 | rww | Yes | Vel. unit (r/min) |
| ----- | --- | --- | --------------- | --- | --- | ------ | --- | --- | ----------------- |
606Fh 00h Velocity threshold UINT16 rww Yes Vel. unit (r/min)
| 6072h | 00h |     | Max torque          |     |     | UINT16 | rww | Yes | 1=0.1%           |
| ----- | --- | --- | ------------------- | --- | --- | ------ | --- | --- | ---------------- |
| 6077h | 00h |     | Torque actual value |     |     | INT16  | ro  | Yes | 1=0.1%           |
| 607Ah | 00h |     | Target position     |     |     | INT32  | rww | Yes | Pos. unit (step) |
Software position limit
Highest Sub-index 
|       | 00h |     |                    |     |     | UINT8 | c   | No  | –                |
| ----- | --- | --- | ------------------ | --- | --- | ----- | --- | --- | ---------------- |
| 607Dh |     |     | Supported          |     |     |       |     |     |                  |
|       | 01h |     | Min position limit |     |     | INT32 | rww | Yes | Pos. unit (step) |
|       | 02h |     | Max position limit |     |     | INT32 | rww | Yes | Pos. unit (step) |
6083h 00h Profile acceleration UINT32 rww Yes Acc. unit ((r/min)/s)
6084h 00h Profile deceleration UINT32 rww Yes Acc. unit ((r/min)/s)
6085h 00h Quick stop deceleration UINT32 rww Yes Acc. unit ((r/min)/s)
| 60FFh | 00h |     | Target velocity |     |     | INT32 | rww | Yes | Vel. unit (r/min) |
| ----- | --- | --- | --------------- | --- | --- | ----- | --- | --- | ----------------- |
7.2.3  Controlword of the Profile Velocity Mode
| 15 14   | 13   | 12    | 11    | 10      | 9   | 8 7 6   | 5       | 4 3 | 2 1 0    |
| ------- | ---- | ----- | ----- | ------- | --- | ------- | ------- | --- | -------- |
| RSV [2] | PVCM | PVPOS | PVNSP | RSV [2] |     | HALT FR | RSV [3] | EO  | QS EV SO |
MSB LSB
| Bit | Notation |                |     | Meaning |     |     |     | Description |     |
| --- | -------- | -------------- | --- | ------- | --- | --- | --- | ----------- | --- |
| 0   | SO       | Switch on      |     |         |     |     |     |             |     |
| 1   | EV       | Enable voltage |     |         |     |     |     |             |     |
Status Machine control commands
| 2      | QS  | Quick stop       |     |     |     |                     |     |     |     |
| ------ | --- | ---------------- | --- | --- | --- | ------------------- | --- | --- | --- |
| 3      | EO  | Enable operation |     |     |     |                     |     |     |     |
| 4 to 6 | RSV | Reserved         |     |     |     | Reserved            |     |     |     |
| 7      | FR  | Fault reset      |     |     |     | 0 -> 1: Alarm reset |     |     |     |
0: Executes or continues operation. 
| 8   | HALT | Halt |     |     |     |     |     |     |     |
| --- | ---- | ---- | --- | --- | --- | --- | --- | --- | --- |
1: Stops the motor according to halt option code (605Dh).
| 9, 10 | RSV | Reserved |     |     |     | Reserved |     |     |     |
| ----- | --- | -------- | --- | --- | --- | -------- | --- | --- | --- |
New set point of PV using a 
11 PVNSP trajectory generator for  0 -> 1: Starts the next PV-positioning operation immediately.
positioning.
|     |       | PV use a trajectory       |     |     |     | 0: A trajectory generator for positioning is disabled.  |     |     |     |
| --- | ----- | ------------------------- | --- | --- | --- | ------------------------------------------------------- | --- | --- | --- |
| 12  | PVPOS |                           |     |     |     |                                                         |     |     |     |
|     |       | generator for positioning |     |     |     | 1: A trajectory generator for positioning is enabled.   |     |     |     |
0: Motion extension 
| 13  | PVCM | PV control mode |     |     |     |     |     |     |     |
| --- | ---- | --------------- | --- | --- | --- | --- | --- | --- | --- |
1: Normal
| 14 to 15 | RSV | Reserved |     |     |     | Reserved |     |     |     |
| -------- | --- | -------- | --- | --- | --- | -------- | --- | --- | --- |
Note: If the Remote bit of the statusword (6041h: bit 9) is 0, the Controlword other than “Quick stop”, “Fault reset”, and 
“Halt” are invalid.
38

## Page 39

Operation mode
7.2.4  Statusword of the Profile Velocity Mode
| 15 14 | 13 12 | 11 10 | 9 8 | 7 6 | 5 4 | 3 2 | 1 0 |
| ----- | ----- | ----- | --- | --- | --- | --- | --- |
TLC RSV RSV SPD ILA TR RM DPRDY WNG SOD QS VE FAULT OE SO RTSO
| MSB          |                      |     |                             |     |             |     | LSB |
| ------------ | -------------------- | --- | --------------------------- | --- | ----------- | --- | --- |
| Bit Notation | Meaning              |     |                             |     | Description |     |     |
| 0 RTSO       | Ready to switch on   |     |                             |     |             |     |     |
| 1            | SO Switch on         |     |                             |     |             |     |     |
| 2            | OE Operation enabled |     |                             |     |             |     |     |
| 3 FAULT      | Fault                |     | Current state of the driver |     |             |     |     |
| 4            | VE Voltage enabled   |     |                             |     |             |     |     |
| 5            | QS Quick stop        |     |                             |     |             |     |     |
| 6 SOD        | Switch on disabled   |     |                             |     |             |     |     |
0: No alarm occurred 
| 7 WNG | Warning |     |     |     |     |     |     |
| ----- | ------- | --- | --- | --- | --- | --- | --- |
1: Alarm occurred
|     | Drive profile  |     | 0: Not ready for operation  |     |     |     |     |
| --- | -------------- | --- | --------------------------- | --- | --- | --- | --- |
8 DPRDY
|     | operation ready |     | 1: Ready for operation |     |     |     |     |
| --- | --------------- | --- | ---------------------- | --- | --- | --- | --- |
0: Controlword is not processed. * 
| 9   | RM Remote |     |     |     |     |     |     |
| --- | --------- | --- | --- | --- | --- | --- | --- |
1: Controlword is processed.
0: Halt (bit 8 in controlword) = 0: The target speed has not been reached. 
Halt (bit 8 in controlword) = 1: The motor is decelerating.
| 10  | TR Target reached |     |     |     |     |     |     |
| --- | ----------------- | --- | --- | --- | --- | --- | --- |
1: Halt (bit 8 in controlword) = 0: The target speed was reached.  
Halt (bit 8 in controlword) = 1: The motor is stopped.
The internal limit is activated in the following cases: 
- The software limit was activated. 
- The FW-LS or RV-LS signal was activated. 
| 11  | ILA Internal limit active |     |     |     |     |     |     |
| --- | ------------------------- | --- | --- | --- | --- | --- | --- |
- The FW-BLK or RV-BLK signal was activated. 
- The STOP or QSTOP signal was activated. 
- The CLR signal was activated.
0: The speed is not 0. 
| 12 SPD | Speed |     |     |     |     |     |     |
| ------ | ----- | --- | --- | --- | --- | --- | --- |
1: The speed is 0.
| 13 RSV | Reserved |     | Reserved |     |     |     |     |
| ------ | -------- | --- | -------- | --- | --- | --- | --- |
| 14 RSV | Reserved |     | Reserved |     |     |     |     |
The torque limit control is activated in the following cases: 
15 TLC Torque limit control - The actual torque reaches the maximum output torque. 
- The actual torque reaches the torque limiting value.
* The Remote (bit 9) is “0” when any of the following conditions. 
- The S-ON signal is active. 
- Remote operation, data writing, or I/O test is executed with the support soft.
39

## Page 40

Operation mode
7.2.5 Operation in the Profile Velocity Mode
z PV use a trajectory generator for positioning [When the (6040h: bit12) is 0]
Velocity
Profile acceleration
V2 Profile deceleration
(6083h)
(6084h)
V1
Target velocity 0 V1 V2 0 V1
(6082h)
Halt
(6040h: bit8)
SPEED
(6041h: bit12)
Target reached
(6041h: bit10)
z PV use a trajectory generator for positioning [When the (6040h: bit12) is 1]
This operation treats absolute value of the target position (607Ah) as a relative movement distance.
The rotary direction is according to sign of the target velocity (60FFh).
Profile acceleration
Velocity (6083h) Profile deceleration
(6084h)
V1
Profile deceleration
(6084h)
V2
Target position
(607Ah)
Target velocity
V1 V2
(6082h)
New set point of PV
(6040h: bit11)
SPEED
(6041h: bit12)
Target reached
(6041h: bit10)
40

## Page 41

Operation mode
7.3 Profile Position Mode (pp)
7.3.1 General Information
The Profile Position Mode is used to position to the target position at the profile velocity and the profile acceleration.
The following figure shows the block diagram for the Profile Position Mode.
Max torque (6072h)
Target position (607Ah)
Software position limit (607Dh) Position
Pos. unit
Positioning option code (60F2h) Limit function
Profile velocity (6081h)
End velocity (6082h) Vel. unit
Profile acceleration (6083h)
Trajectory Position Speed Torque
Profile deceleration (6084h) Generator control control control
Motor
Quick stop deceleration (6085h)
Quick stop option code (605Ah)
Halt option code (605Dh)
Encoder
Torque actual value (6077h)
Velocity actual value (606Ch)
Vel. unit
Position actual value (6064h)
Pos. unit
Position window (6067h)
Target reached in statusword Position reached
window
comparator
Following error actual value (60F4h)
Pos. unit
Following error window (6065h)
Following error in statusword Following error
window
comparator
41

## Page 42

Operation mode
7.3.2  Related Objects
| Index | Sub-index                  | Name | Type Access | PDO mapping | Unit (default) |
| ----- | -------------------------- | ---- | ----------- | ----------- | -------------- |
| 6040h | 00h Controlword            |      | UINT16 rww  | Yes         | –              |
| 6041h | 00h Statusword             |      | UINT16 ro   | Yes         | –              |
| 605Ah | 00h Quick stop option code |      | INT16 rw    | No          | –              |
| 605Dh | 00h Halt option code       |      | INT16 rw    | No          | –              |
6062h 00h Position demand value INT32 ro Yes Pos. unit (step)
6064h 00h Position actual value INT32 ro Yes Pos. unit (step)
6065h 00h Following error window UINT32 rww Yes Pos. unit (step)
| 6067h | 00h Position window |     | UINT32 rww | Yes | Pos. unit (step) |
| ----- | ------------------- | --- | ---------- | --- | ---------------- |
606Ch 00h Velocity actual value INT32 ro Yes Vel. unit (r/min)
| 6072h | 00h Max torque          |     | UINT16 rww | Yes | 1=0.1%           |
| ----- | ----------------------- | --- | ---------- | --- | ---------------- |
| 6077h | 00h Torque actual value |     | INT16 ro   | Yes | 1=0.1%           |
| 607Ah | 00h Target position     |     | INT32 rww  | Yes | Pos. unit (step) |
Software position limit
Highest Sub-index 
|       | 00h                    |     | UINT8 c    | No  | –                 |
| ----- | ---------------------- | --- | ---------- | --- | ----------------- |
| 607Dh | Supported              |     |            |     |                   |
|       | 01h Min position limit |     | INT32 rww  | Yes | Pos. unit (step)  |
|       | 02h Max position limit |     | INT32 rww  | Yes | Pos. unit (step)  |
| 6081h | 00h Profile velocity   |     | UINT32 rww | Yes | Vel. unit (r/min) |
| 6082h | 00h End velocity       |     | UINT32 rww | Yes | Vel. unit (r/min) |
6083h 00h Profile acceleration UINT32 rww Yes Acc. unit ((r/min)/s)
6084h 00h Profile deceleration UINT32 rww Yes Acc. unit ((r/min)/s)
6085h 00h Quick stop deceleration UINT32 rww Yes Acc. unit ((r/min)/s)
| 60F2h | 00h  Positioning option code |     | UINT16 rww | Yes | –   |
| ----- | ---------------------------- | --- | ---------- | --- | --- |
42

## Page 43

Operation mode
7.3.3  Controlword of the Profile Position Mode
| 15  | 14  | 13       | 12             | 11      | 10  | 9 8       | 7 6    | 5 4         | 3 2   | 1 0   |
| --- | --- | -------- | -------------- | ------- | --- | --------- | ------ | ----------- | ----- | ----- |
|     |     | RSV [6]  |                |         |     | COSP HALT | FR REL | IMM NSP     | EO QS | EV SO |
| MSB |     |          |                |         |     |           |        |             |       | LSB   |
|     | Bit | Notation |                | Meaning |     |           |        | Description |       |       |
|     | 0   | SO       | Switch on      |         |     |           |        |             |       |       |
|     | 1   | EV       | Enable voltage |         |     |           |        |             |       |       |
Status Machine control commands
|     | 2   | QS  | Quick stop       |     |     |     |     |     |     |     |
| --- | --- | --- | ---------------- | --- | --- | --- | --- | --- | --- | --- |
|     | 3   | EO  | Enable operation |     |     |     |     |     |     |     |
|     | 4   | NSP | New set point    |     |     |     |     |     |     |     |
Refer to following table.
|     | 5   | IMM | Change set immediately |     |     |     |     |     |     |     |
| --- | --- | --- | ---------------------- | --- | --- | --- | --- | --- | --- | --- |
0: Treats the target position as an absolute value.
1: Treats the target position as a relative value. 
|     | 6   | REL | Abs/rel |     |     |     |     |     |     |     |
| --- | --- | --- | ------- | --- | --- | --- | --- | --- | --- | --- |
(Treats it as the movement distance from the current target 
position.)
|     | 7   | FR  | Fault reset |     |     | 0 -> 1: Alarm reset |     |     |     |     |
| --- | --- | --- | ----------- | --- | --- | ------------------- | --- | --- | --- | --- |
0: Executes or continues positioning. 
|     | 8   | HALT | Halt |     |     |     |     |     |     |     |
| --- | --- | ---- | ---- | --- | --- | --- | --- | --- | --- | --- |
1: Stops the motor according to halt option code (605Dh).
|          | 9   | COSP | Change on set point |     |     | Not supported. It must be set to “0”. |     |     |     |     |
| -------- | --- | ---- | ------------------- | --- | --- | ------------------------------------- | --- | --- | --- | --- |
| 10 to 15 |     | RSV  | Reserved            |     |     | Reserved                              |     |     |     |     |
Change set 
New set point
| immediately |       |     |     |       |     |     |     | Description |     |     |
| ----------- | ----- | --- | --- | ----- | --- | --- | --- | ----------- | --- | --- |
|             | Bit 5 |     |     | Bit 4 |     |     |     |             |     |     |
Starts the next positioning operation after the current positioning operation 
|     | 0   |     | 0  1 |     |     |     |     |     |     |     |
| --- | --- | --- | ----- | --- | --- | --- | --- | --- | --- | --- |
is completed (i.e., after the target is reached).
|     | 1   |     | 0  1 |     | Starts the next positioning operation immediately. |     |     |     |     |     |
| --- | --- | --- | ----- | --- | -------------------------------------------------- | --- | --- | --- | --- | --- |
Note: If the Remote bit of the Statusword (6041h: bit 9) is 0, the Controlword other than “Quick stop”, “Fault reset”, and 
“Halt” are invalid.
43

## Page 44

Operation mode
7.3.4  Statusword of the Profile Position Mode
| 15 14 | 13 12 | 11 10 | 9 8 | 7 6 | 5 4 | 3 2 | 1 0 |
| ----- | ----- | ----- | --- | --- | --- | --- | --- |
TLC RSV ERROR SPA ILA TR RM DPRDY WNG SOD QS VE FAULT OE SO RTSO
| MSB          |         |     |     |     |             |     | LSB |
| ------------ | ------- | --- | --- | --- | ----------- | --- | --- |
| Bit Notation | Meaning |     |     |     | Description |     |     |
Ready to switch 
0 RTSO
on
| 1   | SO Switch on |     |     |     |     |     |     |
| --- | ------------ | --- | --- | --- | --- | --- | --- |
Operation 
2 OE
enabled
Current state of the driver
| 3 FAULT | Fault              |     |     |     |     |     |     |
| ------- | ------------------ | --- | --- | --- | --- | --- | --- |
| 4       | VE Voltage enabled |     |     |     |     |     |     |
| 5       | QS Quick stop      |     |     |     |     |     |     |
Switch on 
6 SOD
disabled
0: No alarm occurred 
| 7 WNG | Warning |     |     |     |     |     |     |
| ----- | ------- | --- | --- | --- | --- | --- | --- |
1: Alarm occurred
|     | Drive profile  | 0: Not ready for operation  |     |     |     |     |     |
| --- | -------------- | --------------------------- | --- | --- | --- | --- | --- |
8 DPRDY
|     | operation ready | 1: Ready for operation |     |     |     |     |     |
| --- | --------------- | ---------------------- | --- | --- | --- | --- | --- |
0: Controlword is not processed. * 
| 9   | RM Remote |     |     |     |     |     |     |
| --- | --------- | --- | --- | --- | --- | --- | --- |
1: Controlword is processed.
0: Halt (bit 8 in controlword) = 0: The target position has not been reached. 
Halt (bit 8 in controlword) = 1: The motor is decelerating.
| 10  | TR Target reached |     |     |     |     |     |     |
| --- | ----------------- | --- | --- | --- | --- | --- | --- |
1: Halt (bit 8 in controlword) = 0: The target position was reached. 
Halt (bit 8 in controlword) = 1: The motor is stopped.
The internal limit is activated in the following cases: 
- The software limit was activated. 
Internal limit 
| 11  | ILA | - The FW-LS or RV-LS signal was activated.  |     |     |     |     |     |
| --- | --- | ------------------------------------------- | --- | --- | --- | --- | --- |
active
- The FW-BLK or RV-BLK signal was activated. 
- The STOP, QSTOP, or CLR signal was activated.
0: Processing of previous set-point (reference) was completed and the driver 
|     | Set point  |     | is waiting for a new set-point. |     |     |     |     |
| --- | ---------- | --- | ------------------------------- | --- | --- | --- | --- |
12 SPA
acknowledge 1: Processing the previous set-point is still in process or a set-point was 
acknowledged.
0: No following error 
| 13 ERROR | Following error |     |     |     |     |     |     |
| -------- | --------------- | --- | --- | --- | --- | --- | --- |
1: Following error
| 14 RSV | Reserved | Reserved |     |     |     |     |     |
| ------ | -------- | -------- | --- | --- | --- | --- | --- |
The torque limit control is activated in the following cases: 
Torque limit 
15 TLC - The actual torque reaches the maximum output torque. 
control
- The actual torque reaches the torque limiting value.
* The Remote (bit 9) is “0” when any of the following conditions. 
- The S-ON signal is active. 
- Remote operation, data writing, or I/O test is executed with the support soft.
44

## Page 45

Operation mode
7.3.5 Operation in the Profile Position Mode
Positioning operation is started when the “Target position (607Ah)” is set and the “New set point (6040h: bit4)” is set to 1.
Velocity
Profile acceleration
Profile deceleration
(6083h)
(6084h)
Profile velocity
(6081h)
Target position
(607Ah)
End velocity
(6082h)
Target position
(607Ah)
New set point
(6040h: bit4)
Set point acknowledge
(6041h: bit12)
Target reached
(6041h: bit10)
z Single set-point [When the “Change set immediately (6040h: bit5)” is 1]
If the “New set point (6040h: bit4)” is set during operation, the new operation command is applied immediately.
Velocity
Target position
(607Ah)
New set point
(6040h: bit4)
Set point acknowledge
(6041h: bit12)
Effective target position
Target reached
(6041h: bit10)
z Set of set-points [When the “Change set immediately (6040h: bit5)” is 0]
If the “New set point (6040h: bit4)” is set during operation, the new operation command is stored. When the present
operation is complete, the stored new operation command is started.
Velocity
Target position
(607Ah)
New set point
(6040h: bit4)
Set point acknowledge
(6041h: bit12)
Effective target position
Target reached
(6041h: bit10)
45

## Page 46

Operation mode
7.3.6  Positioning option code (60F2h)
| Bit | 15       | 14       | 13                           | 12 11 | 10        | 9 8 | 7                         | 6 5 4       | 3 2 | 1 0 |
| --- | -------- | -------- | ---------------------------- | ----- | --------- | --- | ------------------------- | ----------- | --- | --- |
|     | PUSH     | RSV [3]  |                              |       | IPOPT [4] |     | RADO                      | RRO         | CIO | RO  |
|     | MSB      |          |                              |       |           |     |                           |             |     | LSB |
|     | Bit      | Notation |                              |       | Meaning   |     |                           | Description |     |     |
|     | 0, 1     | RO       | Relative option              |       |           |     | Refer to following table. |             |     |     |
|     | 2, 3     | CIO      | Change immediately option    |       |           |     | Not supported.            |             |     |     |
|     | 4, 5     | RRO      | Request-response option      |       |           |     | Not supported.            |             |     |     |
|     | 6, 7     | RADO     | Rotary axis direction option |       |           |     | Refer to following table. |             |     |     |
|     | 8 to 11  | IPOPT    | IP option                    |       |           |     | Not supported.            |             |     |     |
|     | 12 to 14 | RSV      | Reserved                     |       |           |     | Reserved                  |             |     |     |
|     | 15       | PUSH     | Push-motion                  |       |           |     | Refer to following table. |             |     |     |
Bits in 
|     | Controlword  |     | Bits in Positioning option code (60F2h) |     |     |     |     |     |     |     |
| --- | ------------ | --- | --------------------------------------- | --- | --- | --- | --- | --- | --- | --- |
(6040h)
Operation Mode
|     |           | Push-  |     | Rotary axis direction  |       |                 |       |     |     |     |
| --- | --------- | ------ | --- | ---------------------- | ----- | --------------- | ----- | --- | --- | --- |
|     | Abs / rel |        |     |                        |       | Relative option |       |     |     |     |
|     |           | motion |     | option                 |       |                 |       |     |     |     |
|     | Bit 6     | Bit 15 |     | Bit 7                  | Bit 6 | Bit 1           | Bit 0 |     |     |     |
Absolute positioning/Wrap absolute 
|     | 0   |     | 0   | 0   | 0   | X   | X   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
positioning *
Wrap reverse direction absolute 
|     | 0   |     | 0   | 0   | 1   | X   | X   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
positioning *
Wrap forward direction absolute 
|     | 0   |     | 0   | 1   | 0   | X   | X   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
positioning *
|     | 0   |     | 0   | 1   | 1   | X   | X   | Wrap proximity positioning * |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | ---------------------------- | --- | --- |
Absolute positioning push-motion/Wrap 
|     | 0   |     | 1   | 0   | 0   | X   | X   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
absolute push-motion *
|     | 0   |     | 1   | 0   | 1   | X   | X   | Wrap reverse direction push-motion * |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | ------------------------------------ | --- | --- |
|     | 0   |     | 1   | 1   | 0   | X   | X   | Wrap forward direction push-motion * |     |     |
|     | 0   |     | 1   | 1   | 1   | X   | X   | Wrap proximity push-motion *         |     |     |
Incremental positioning (based on target 
|     | 1   |     | 0   | 0   | 0   | 0   | 0   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
position)
Incremental positioning (based on demand 
|     | 1   |     | 0   | 0   | 0   | 0   | 1   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
position)
Incremental positioning (based on actual 
|     | 1   |     | 0   | 0   | 0   | 1   | 0   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
position)
|     | 1   |     | 0   | 0   | 0   | 1   | 1   | Reserved |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | -------- | --- | --- |
Incremental positioning push-motion 
|     | 1   |     | 1   | 0   | 0   | 0   | 0   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
(based on target position)
Incremental positioning push-motion 
|     | 1   |     | 1   | 0   | 0   | 0   | 1   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
(based on command position)
Incremental positioning push-motion 
|     | 1   |     | 1   | 0   | 0   | 1   | 0   |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
(based on actual position)
|     | 1   |     | 1   | 0   | 0   | 1   | 1   | Reserved |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | -------- | --- | --- |
* To do this, Object 607Bh(Position range limit) must have set.
Bits marked by an X are irrelevant.
Refer to the following for details on the operation mode.
- OPERATING MANUAL BLV Series R Type Function Edition
46

## Page 47

Operation mode
7.4  Profile Torque Mode (tq)
7.4.1  General Information
The Profile Torque Mode allows to transmit the target torque value, which is processed via the trajectory generator. 
The torque slope is required.
The following figure shows the block diagram for the Profile Torque Mode.
Target torque (6071h)
Torque slope (6087h)
Max torque (6072h)
| Profile velocity (6081h) |     |     |                                  |     | Torque  |
| ------------------------ | --- | --- | -------------------------------- | --- | ------- |
|                          |     |     | Trajectory Torque demand (6074h) |     |         |
|                          |     |     | Generator                        |     | control |
Quick stop deceleration (6085h) Motor
Quick stop option code (605Ah)
Halt option code (605Dh)
  Encoder
Torque actual value (6077h)
| Target reached in statusword | Target reached |     |     |     |     |
| ---------------------------- | -------------- | --- | --- | --- | --- |
comparator
Velocity actual value (606Ch)
Vel. unit
Position actual value (6064h)
Pos. unit
7.4.2  Related Objects
| Index | Sub-index                  | Name | Type Access | PDO mapping | Unit (default) |
| ----- | -------------------------- | ---- | ----------- | ----------- | -------------- |
| 6040h | 00h Controlword            |      | UINT16 rww  | Yes         | –              |
| 6041h | 00h Statusword             |      | UINT16 ro   | Yes         | –              |
| 605Ah | 00h Quick stop option code |      | INT16 rw    | No          | –              |
| 605Dh | 00h Halt option code       |      | INT16 rw    | No          | –              |
6064h 00h Position actual value INT32 ro Yes Pos. unit (step)
606Ch 00h Velocity actual value INT32 ro Yes Vel. unit (r/min)
| 6071h | 00h Target torque       |     | INT16 rww  | Yes | 1=0.1%            |
| ----- | ----------------------- | --- | ---------- | --- | ----------------- |
| 6072h | 00h Max torque          |     | UINT16 rww | Yes | 1=0.1%            |
| 6074h | 00h Torque demand       |     | INT16 ro   | Yes | 1=0.1%            |
| 6077h | 00h Torque actual value |     | INT16 ro   | Yes | 1=0.1%            |
| 6081h | 00h Profile velocity    |     | UINT32 rww | Yes | Vel. unit (r/min) |
6085h 00h Quick stop deceleration UINT32 rww Yes Acc. unit ((r/min)/s)
| 6087h | 00h Torque slope |     | UINT32 rww | Yes | 1=0.1%/s |
| ----- | ---------------- | --- | ---------- | --- | -------- |
47

## Page 48

Operation mode
7.4.3  Controlword of the Profile Torque Mode
| 15 14 | 13 12             | 11 10   | 9 8  | 7 6 | 5 4         | 3 2   | 1 0   |
| ----- | ----------------- | ------- | ---- | --- | ----------- | ----- | ----- |
|       | RSV [7]           |         | HALT | FR  | RSV [3]     | EO QS | EV SO |
| MSB   |                   |         |      |     |             |       | LSB   |
| Bit   | Notation          | Meaning |      |     | Description |       |       |
| 0     | SO Switch on      |         |      |     |             |       |       |
| 1     | EV Enable voltage |         |      |     |             |       |       |
Status Machine control commands
| 2      | QS Quick stop       |     |                     |     |     |     |     |
| ------ | ------------------- | --- | ------------------- | --- | --- | --- | --- |
| 3      | EO Enable operation |     |                     |     |     |     |     |
| 4 to 6 | RSV Reserved        |     | Reserved            |     |     |     |     |
| 7      | FR Fault reset      |     | 0 -> 1: Alarm reset |     |     |     |     |
0: Executes or continues operation. 
| 8   | HALT Halt |     |     |     |     |     |     |
| --- | --------- | --- | --- | --- | --- | --- | --- |
1: Stops the motor according to torque slope (6087h).
| 9 to 15 | RSV Reserved |     | Reserved |     |     |     |     |
| ------- | ------------ | --- | -------- | --- | --- | --- | --- |
48

## Page 49

Operation mode
7.4.4  Statusword of the Profile Torque Mode
| 15 14 | 13       | 12 11   | 10 9        | 8 7 6   | 5 4         | 3 2      | 1 0     |
| ----- | -------- | ------- | ----------- | ------- | ----------- | -------- | ------- |
| TLC   | RSV [3]  | ILA     | TR RM DPRDY | WNG SOD | QS VE       | FAULT OE | SO RTSO |
| MSB   |          |         |             |         |             |          | LSB     |
| Bit   | Notation | Meaning |             |         | Description |          |         |
Ready to switch 
| 0   | RTSO |     |     |     |     |     |     |
| --- | ---- | --- | --- | --- | --- | --- | --- |
on
| 1   | SO  | Switch on |     |     |     |     |     |
| --- | --- | --------- | --- | --- | --- | --- | --- |
Operation 
| 2   | OE  |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- |
enabled
Current state of the driver
| 3   | FAULT | Fault           |     |     |     |     |     |
| --- | ----- | --------------- | --- | --- | --- | --- | --- |
| 4   | VE    | Voltage enabled |     |     |     |     |     |
| 5   | QS    | Quick stop      |     |     |     |     |     |
Switch on 
| 6   | SOD |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- |
disabled
0: No alarm occurred 
| 7   | WNG | Warning |     |     |     |     |     |
| --- | --- | ------- | --- | --- | --- | --- | --- |
1: Alarm occurred
|     |       | Drive profile   | 0: Not ready for operation  |     |     |     |     |
| --- | ----- | --------------- | --------------------------- | --- | --- | --- | --- |
| 8   | DPRDY |                 |                             |     |     |     |     |
|     |       | operation ready | 1: Ready for operation      |     |     |     |     |
0: Controlword is not processed. * 
| 9   | RM  | Remote |     |     |     |     |     |
| --- | --- | ------ | --- | --- | --- | --- | --- |
1: Controlword is processed.
0: Halt (bit 8 in controlword) = 0: The target torque has not been reached. 
Halt (bit 8 in controlword) = 1: The motor is decelerating.
| 10  | TR  | Target reached |     |     |     |     |     |
| --- | --- | -------------- | --- | --- | --- | --- | --- |
1: Halt (bit 8 in controlword) = 0: The target torque was reached. 
Halt (bit 8 in controlword) = 1: The motor is stopped.
The internal limit is activated in the following cases: 
- The software limit was activated. 
|     |     | Internal limit  | - The FW-LS or RV-LS signal was activated.    |     |     |     |     |
| --- | --- | --------------- | --------------------------------------------- | --- | --- | --- | --- |
| 11  | ILA |                 |                                               |     |     |     |     |
|     |     | active          | - The FW-BLK or RV-BLK signal was activated.  |     |     |     |     |
- The STOP, QSTOP, or CLR signal was activated. 
- The CLR signal was activated.
| 12 to 14 | RSV | Reserved | Reserved |     |     |     |     |
| -------- | --- | -------- | -------- | --- | --- | --- | --- |
The torque limit control is activated in the following cases: 
Torque limit 
15 TLC - The actual torque reaches the maximum output torque. 
control
- The actual torque reaches the torque limiting value.
* The Remote (bit 9) is “0” when any of the following conditions. 
- The S-ON signal is active. 
- Remote operation, data writing, or I/O test is executed with the support soft.
49

## Page 50

Operation mode
7.4.5 Operation in the Profile Torque Mode
The Profile Torque Mode updates immediately when Target torque (6071h) is set/changed.
The Profile Torque Mode requires setting Target torque (6071h), Torque slope (6087h).
The driver generates commands as long as HALT (bit 8 in controlword) is “0”.
If HALT (bit 8 in controlword) is set to “1”, the trajectory generation process will set the torque to 0.
The maximum velocity in this mode can be set with Profile velocity (6081h).
Torque slope Torque slope
(6087h) (6087h)
Torque
T1
Torque slope Torque slope
(6087h) (6087h)
T2
0
Target torque
0 T1 T2 0
(6071h)
Halt
(6040h: bit8)
Target reached
(6041h: bit10)
50

## Page 51

Operation mode
7.5  Homing Mode (hm)
7.5.1  General Information
The following figure shows the relationship between the input objects and the output objects in the Homing Mode. 
You can specify the speeds, acceleration rate, and homing method. 
You can also use home offset to offset zero in the user coordinate system from the home position.
Controlword (6040h)
Homing method (6098h)
Statusword (6041h)
Homing speeds (6084h)
Homing acceleration(609Ah)
| Home offset (607Ch) |     | Homing |     |     |     |
| ------------------- | --- | ------ | --- | --- | --- |
  method
Position demand value (6062h)
JOG/HOME torque limiting value (415Fh)
(HOME)Starting velocity (4163h)
(HOME)Backward steps in 2 sensor 
home-seeking (4169h)
7.5.2  Related Objects
| Index | Sub-index       | Name | Type Access | PDO mapping | Unit (default) |
| ----- | --------------- | ---- | ----------- | ----------- | -------------- |
| 6040h | 00h Controlword |      | UINT16 rww  | Yes         | –              |
| 6041h | 00h Statusword  |      | UINT16 ro   | Yes         | –              |
6062h 00h Position demand value INT32 ro Yes Pos. unit (step)
| 607Ch | 00h Home offset   |     | INT32 rww | Yes | Pos. unit (step) |
| ----- | ----------------- | --- | --------- | --- | ---------------- |
| 6098h | 00h Homing method |     | INT8 rww  | Yes | –                |
Homing speeds
Highest sub-index 
|     | 00h |     | UINT8 c | No  | –   |
| --- | --- | --- | ------- | --- | --- |
supported
| 6099h | Speed during search for  |     |            |     |                   |
| ----- | ------------------------ | --- | ---------- | --- | ----------------- |
|       | 01h                      |     | UINT32 rww | Yes | Vel. unit (r/min) |
switch
Speed during search for 
|     | 02h |     | UINT32 rww | Yes | Vel. unit (r/min) |
| --- | --- | --- | ---------- | --- | ----------------- |
zero
609Ah 00h Homing acceleration UINT32 rww Yes Acc. unit ((r/min)/s)
JOG/HOME torque limiting 
| 415Fh | 00h |     | UINT32 rww | Yes | 1=0.1% |
| ----- | --- | --- | ---------- | --- | ------ |
value
4163h 00h (HOME) Starting velocity UINT32 rww Yes Vel. unit (r/min)
(HOME) Backward steps in 
| 4169h | 00h |     | UINT32 rww | Yes | Pos. unit (step) |
| ----- | --- | --- | ---------- | --- | ---------------- |
2 sensor home-seeking
51

## Page 52

Operation mode
7.5.3  Controlword of the Homing Mode
| 15 14 | 13 12             | 11 10   | 9 8  | 7 6        | 5 4         | 3 2   | 1 0   |
| ----- | ----------------- | ------- | ---- | ---------- | ----------- | ----- | ----- |
|       | RSV [7]           |         | HALT | FR RSV [2] | HOS         | EO QS | EV SO |
| MSB   |                   |         |      |            |             |       | LSB   |
| Bit   | Notation          | Meaning |      |            | Description |       |       |
| 0     | SO Switch on      |         |      |            |             |       |       |
| 1     | EV Enable voltage |         |      |            |             |       |       |
Status Machine control commands
| 2   | QS Quick stop       |     |     |     |     |     |     |
| --- | ------------------- | --- | --- | --- | --- | --- | --- |
| 3   | EO Enable operation |     |     |     |     |     |     |
0: Does not start homing procedure. 
| 4   | HOS Homing operation start |     |     |     |     |     |     |
| --- | -------------------------- | --- | --- | --- | --- | --- | --- |
1: Starts or continues homing procedure.
| 5, 6 | RSV Reserved   |     | Reserved            |     |     |     |     |
| ---- | -------------- | --- | ------------------- | --- | --- | --- | --- |
| 7    | FR Fault reset |     | 0 -> 1: Alarm reset |     |     |     |     |
0: Enable bit 4. 
| 8   | HALT Halt |     |     |     |     |     |     |
| --- | --------- | --- | --- | --- | --- | --- | --- |
1: Stops the motor according to halt option code (605Dh).
| 9 to 15 | RSV Reserved |     | Reserved |     |     |     |     |
| ------- | ------------ | --- | -------- | --- | --- | --- | --- |
Note: If the Remote bit of the statusword (6041h: bit 9) is 0, the Controlword other than “Quick stop”, “Fault reset”, and 
“Halt” are invalid.
52

## Page 53

Operation mode
7.5.4  Statusword of the Homing Mode
| 15  | 14       | 13                   | 12  | 11 10   | 9        | 8 7 6                       | 5   | 4           | 3 2      | 1 0     |
| --- | -------- | -------------------- | --- | ------- | -------- | --------------------------- | --- | ----------- | -------- | ------- |
| TLC | RSV      | HE                   | HA  | ILA TR  | RM DPRDY | WNG SOD                     | QS  | VE          | FAULT OE | SO RTSO |
| MSB |          |                      |     |         |          |                             |     |             |          | LSB     |
| Bit | Notation |                      |     | Meaning |          |                             |     | Description |          |         |
| 0   | RTSO     | Ready to switch on   |     |         |          |                             |     |             |          |         |
| 1   |          | SO Switch on         |     |         |          |                             |     |             |          |         |
| 2   |          | OE Operation enabled |     |         |          |                             |     |             |          |         |
| 3   | FAULT    | Fault                |     |         |          | Current state of the driver |     |             |          |         |
| 4   |          | VE Voltage enabled   |     |         |          |                             |     |             |          |         |
| 5   |          | QS Quick stop        |     |         |          |                             |     |             |          |         |
| 6   | SOD      | Switch on disabled   |     |         |          |                             |     |             |          |         |
0: No alarm occurred 
| 7   | WNG | Warning |     |     |     |     |     |     |     |     |
| --- | --- | ------- | --- | --- | --- | --- | --- | --- | --- | --- |
1: Alarm occurred
0: Not ready for operation 
| 8   | DPRDY | Drive profile operation ready |     |     |     |     |     |     |     |     |
| --- | ----- | ----------------------------- | --- | --- | --- | --- | --- | --- | --- | --- |
1: Ready for operation
0: Controlword is not processed. * 
| 9   |     | RM Remote |     |     |     |     |     |     |     |     |
| --- | --- | --------- | --- | --- | --- | --- | --- | --- | --- | --- |
1: Controlword is processed.
| 10  |     | TR Target reached |     |     |     | Refer to following table. |     |     |     |     |
| --- | --- | ----------------- | --- | --- | --- | ------------------------- | --- | --- | --- | --- |
The internal limit is activated in the following cases: 
- The software limit was activated. 
- The FW-LS or RV-LS signal was activated. 
| 11  |     | ILA Internal limit active |     |     |     |     |     |     |     |     |
| --- | --- | ------------------------- | --- | --- | --- | --- | --- | --- | --- | --- |
- The FW-BLK or RV-BLK signal was activated. 
- The STOP or QSTOP signal was activated. 
- The CLR signal was activated.
| 12  |     | HA Homing attained |     |     |     |     |     |     |     |     |
| --- | --- | ------------------ | --- | --- | --- | --- | --- | --- | --- | --- |
Refer to following table.
| 13  |     | HE Homing error |     |     |     |          |     |     |     |     |
| --- | --- | --------------- | --- | --- | --- | -------- | --- | --- | --- | --- |
| 14  |     | RSV Reserved    |     |     |     | Reserved |     |     |     |     |
The torque limit control is activated in the following cases: 
15 TLC Torque limit control - The actual torque reaches the maximum output torque. 
- The actual torque reaches the torque limiting value.
* The Remote (bit 9) is “0” when any of the following conditions. 
- The S-ON signal is active. 
- Remote operation, data writing, or I/O test is executed with the support soft.
| Homing error |     | Homing attained |     |     | Target reached |     |     |     |     |     |
| ------------ | --- | --------------- | --- | --- | -------------- | --- | --- | --- | --- | --- |
Description
|     | Bit 13 |     | Bit 12 |     | Bit 10 |                                  |     |     |     |     |
| --- | ------ | --- | ------ | --- | ------ | -------------------------------- | --- | --- | --- | --- |
|     | 0      |     | 0      |     | 0      | Homing procedure is in progress. |     |     |     |     |
0 0 1 Homing procedure was interrupted or has not yet started.
0 1 0 Homing is attained, but the operation is still in progress.
|     | 0   |     | 1   |     | 1   | Homing procedure was completed successfully.       |     |     |     |     |
| --- | --- | --- | --- | --- | --- | -------------------------------------------------- | --- | --- | --- | --- |
|     | 1   |     | 0   |     | 0   | A homing error occurred and the velocity is not 0. |     |     |     |     |
|     | 1   |     | 0   |     | 1   | A homing error occurred and the velocity is 0.     |     |     |     |     |
|     | 1   |     | 1   |     | 0   |                                                    |     |     |     |     |
Reserved
|     | 1   |     | 1   |     | 1   |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
53

## Page 54

Operation mode
7.5.5 Homing method
The following homing methods are supported.
Homing method Definition
1 Homing on negative limit switch and index pulse
2 Homing on positive limit switch and index pulse
8 Homing on home switch and index pulse and starting in the positive direction
12 Homing on home switch and index pulse and starting in the negative direction
17 Homing on negative limit switch
18 Homing on positive limit switch
24 Homing on home switch and starting in the positive direction
28 Homing on home switch and starting in the negative direction
35, 37 * Homing on current position
Homing method of Orientalmotor specifications.
–1 Refer to the following for details on the operations.
- OPERATING MANUAL BLV Series R Type Function Edition
* 35 and 37 perform the same action
Note: The index pulse is the ZSG-N signal.
7.5.5.1 Method 1: Homing on negative limit switch and index pulse
In this method, homing starts in the negative direction if the negative limit switch is inactive.
After the negative limit switch becomes inactive, the motor rotates to stop according to the value set in the “(HOME)
Backward steps in 2 sensor homing (4169h).”
The home position is the first index pulse that is detected after this moving.
1
Index pulse
(ZSG-N)
Negative limit switch
(RV-LS)
7.5.5.2 Method 2: Homing on positive limit switch and index pulse
In this method, homing starts in the positive direction if the positive limit switch is inactive.
After the positive limit switch becomes inactive, the motor rotates to stop according to the value set in the “(HOME)
Backward steps in 2 sensor homing (4169h).”
The home position is the first index pulse that is detected after this moving.
2
Index pulse
(ZSG-N)
Positive limit switch
(FW-LS)
54

## Page 55

Operation mode
7.5.5.3 Method 8: Homing on home switch and index pulse and starting in the positive direction
In this method, homing starts in the positive direction.
However, if the home switch is already active when homing is started, the initial homing direction depends on the
required edge. The home position will be the index pulse on the rising edge side of the home switch.
If the initial movement direction is away from the home switch, the motor will reverse direction when the limit switch
in the movement direction is input.
8
8
8
Index pulse
(ZSG-N)
Home switch
(HOMES)
Positive limit switch
(FW-LS)
7.5.5.4 Method 12: Homing on home switch and index pulse and starting in the negative direction
In this method, homing starts in the negative direction.
However, if the home switch is already active when homing is started, the initial homing direction depends on the
required edge. The home position will be the index pulse on the rising edge side of the home switch.
If the initial movement direction is away from the home switch, the motor will reverse direction when the limit switch
in the movement direction is input.
12
12
12
Index pulse
(ZSG-N)
Home switch
(HOMES)
Negative limit switch
(RV-LS)
55

## Page 56

Operation mode
7.5.5.5 Method 17: Homing on negative limit switch
In this method, homing starts in the negative direction if the negative limit switch is inactive.
After the negative limit switch becomes inactive, the motor rotates to stop according to the value set in the “(HOME)
Backward steps in 2 sensor homing (4169h).” The stop position will be the home position.
17
Negative limit switch
(RV-LS)
7.5.5.6 Method 18: Homing on positive limit switch
In this method, homing starts in the positive direction if the positive limit switch is inactive.
After the positive limit switch becomes inactive, the motor rotates to stop according to the value set in the “(HOME)
Backward steps in 2 sensor homing (4169h).” The stop position will be the home position.
18
Positive limit switch
(FW-LS)
7.5.5.7 Method 24: Homing on home switch and starting in the positive direction
This method is same as method 8 except that the home position does not depend on the index pulse.
Here, it depends only on changes in the relevant home switch (HOMES) or limit switch (FW-LS).
24
24
24
Home switch
(HOMES)
Positive limit switch
(FW-LS)
56

## Page 57

Operation mode
7.5.5.8 Method 28: Homing on home switch and starting in the negative direction
This method is same as method 12 except that the home position does not depend on the index pulse.
Here, it depends only on changes in the relevant home switch (HOMES) or limit switch (RV-LS).
28
28
28
Home switch
(HOMES)
Negative limit switch
(RV-LS)
7.5.5.9 Method 35, 37: Homing on current position
In this method, the current position is defined as the home position.
You can execute this method even if the drive device is not in the Operation Enabled state.
57

## Page 58

Operation mode
7.6 Touch probe functionality
7.6.1 General Information
You can latch the actual position with the following trigger events.
• Trigger with probe 1 input (USR-LAT-IN0 input signal)
• Trigger with probe 2 input (USR-LAT-IN1 input signal)
• Trigger with ZSG-N output signal
Note: The trigger events must be active for 1 ms or more.
7.6.2 Related Objects
Index Sub-index Name Type Access PDO mapping Unit (default)
60B8h 00h Touch probe function UINT16 rww Yes –
60B9h 00h Touch probe status UINT16 ro Yes –
60BAh 00h Touch probe 1 positive edge INT32 ro Yes Pos. unit (step)
60BBh 00h Touch probe 1 negative edge INT32 ro Yes Pos. unit (step)
60BCh 00h Touch probe 2 positive edge INT32 ro Yes Pos. unit (step)
60BDh 00h Touch probe 2 negative edge INT32 ro Yes Pos. unit (step)
60D5h 00h Touch probe 1 positive edge counter UINT16 ro Yes –
60D6h 00h Touch probe 1 negative edge counter UINT16 ro Yes –
60D7h 00h Touch probe 2 positive edge counter UINT16 ro Yes –
60D8h 00h Touch probe 2 negative edge counter UINT16 ro Yes –
58

## Page 59

Operation mode
7.6.3 Example of Execution Procedure for a Touch Probe
The operation examples of touch probe 1 are shown below.
Trigger first event
Enable touch probe 1
(60B8h: bit 0)
Enable sampling at positive edge of touch probe 1
(60B8h: bit 4)
Latching started Latching started
Touch probe 1 is enabled
(60B9h: bit 0)
Touch probe 1 positive edge position stored
(60B9h: bit 1)
Touch probe 1 positive edge
(60BAh) Latched position 1 Latched position 3
Prove input 1 2 3
Continuous
Enable touch probe 1
(60B8h: bit 0)
Enable sampling at positive edge of touch probe 1
(60B8h: bit 4)
Latching started
Touch probe 1 is enabled
(60B9h: bit 0)
Touch probe 1 positive edge position stored
(60B9h: bit 1)
Touch probe 1 positive edge
(60BAh) p L o a s t i c ti h o e n d 1 p L o a s t i c ti h o e n d 2 Latched position 3
Prove input 1 2 3
59

## Page 60

Object Dictionary
8  Object Dictionary
8.1  Communication Objects
z  1000h: Device Type
This object contains the device type and functionality.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 1000h | 00h | Device type |     | UINT32 |     | c   | No  | 0002 0192h | –   | No  |
| ----- | --- | ----------- | --- | ------ | --- | --- | --- | ---------- | --- | --- |
Data Description
| Bit 31 | 30 29 | 28 27 | 26 25 | 24  | 23  | 22  | 21  | 20 19 18 | 17  | 16  |
| ------ | ----- | ----- | ----- | --- | --- | --- | --- | -------- | --- | --- |
Additional Information
| Bit 15 | 14 13 | 12 11 | 10  | 9 8 | 7   | 6   | 5   | 4 3 2 | 1   | 0   |
| ------ | ----- | ----- | --- | --- | --- | --- | --- | ----- | --- | --- |
Device profile number
| MSB |     |     |     |     |     |     |     |     |     | LSB |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
Additional Information: 2 (0002h) Servo Drive (Brushless motor driver)
Device profile number: 402 (0192h) DS402 drive profile
z  1001h: Error register
If an error bit is set in the manufacturer independent error register, then more detailed information is made available 
in “Pre-defined error field (1003h)”. 
This object is part of the Error Object (Emergency Message). “4.5 Emergency object (EMCY)”
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 1001h | 00h | Error register |     | UINT8 |     | ro  | No  | –   | –   | No  |
| ----- | --- | -------------- | --- | ----- | --- | --- | --- | --- | --- | --- |
Data Description
| Bit 7  | 6 5      | 4                           | 3 2     | 1   | 0   |     |     |     |     |     |
| ------ | -------- | --------------------------- | ------- | --- | --- | --- | --- | --- | --- | --- |
| MS     | RSV [2]  | COM                         | RSV [3] |     | GE  |     |     |     |     |     |
| MSB    |          |                             |         |     | LSB |     |     |     |     |     |
| Bit    | Notation |                             | Meaning |     |     |     |     |     |     |     |
| 0      | GE       | Generic error               |         |     |     |     |     |     |     |     |
| 1 to 3 | RSV      | Reserved                    |         |     |     |     |     |     |     |     |
| 4      | COM      | Communication error         |         |     |     |     |     |     |     |     |
| 5, 6   | RSV      | Reserved                    |         |     |     |     |     |     |     |     |
| 7      | MS       | Manufacturer-specific error |         |     |     |     |     |     |     |     |
60

## Page 61

Object Dictionary
z  1003h: Pre-defined error field
This object contains an error record with up to ten entries.
- The value in Sub-index 00h shows the number of recorded errors.
- The most recent error is shown in Sub-index 01h.
- The error number has the data type UINT32 and is composed of a 16-bit error code and an additional information 
field. The additional information field is not used by this driver.
Note: If no error is present, the value of sub-index 00h is 00h and a read access to sub-index 01h is responded with an 
SDO abort message (abort code: 0800 0024h).
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Pre-defined error field
|     | 00h Number of errors       |     | UINT8  | rw  | No  | 0   | – No |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ---- |
|     | 01h Standard error field 1 |     | UINT32 | ro  | No  | (–) | – No |
|     | 02h Standard error field 2 |     | UINT32 | ro  | No  | (–) | – No |
|     | 03h Standard error field 3 |     | UINT32 | ro  | No  | (–) | – No |
|     | 04h Standard error field 4 |     | UINT32 | ro  | No  | (–) | – No |
1003h
|     | 05h Standard error field 5  |     | UINT32 | ro  | No  | (–) | – No |
| --- | --------------------------- | --- | ------ | --- | --- | --- | ---- |
|     | 06h Standard error field 6  |     | UINT32 | ro  | No  | (–) | – No |
|     | 07h Standard error field 7  |     | UINT32 | ro  | No  | (–) | – No |
|     | 08h Standard error field 8  |     | UINT32 | ro  | No  | (–) | – No |
|     | 09h Standard error field 9  |     | UINT32 | ro  | No  | (–) | – No |
|     | 0Ah Standard error field 10 |     | UINT32 | ro  | No  | (–) | – No |
Data Description
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Additional Information
| Bit 15 | 14 13 12 | 11 10 | 9 8 | 7 6 | 5 4 | 3 2 | 1 0 |
| ------ | -------- | ----- | --- | --- | --- | --- | --- |
Error code
| MSB |     |     |     |     |     |     | LSB |
| --- | --- | --- | --- | --- | --- | --- | --- |
Additional Information: 0 (0000h)
Error code: Refer to “4.5 Emergency object (EMCY)”
61

## Page 62

Object Dictionary
z  1005h: COB-ID SYNC message
This object can be used to change the COB-ID for the SYNC message.
Further, it defines whether the driver generates the SYNC.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0000 0000h 
to 
| 1005h | 00h COB-ID SYNC message |     | UINT32 | rw  | No  |     | – Yes |
| ----- | ----------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0080h)
Data Description
| Bit 31   | 30 29 28             | 27 26 | 25 24   | 23 22 | 21 20  | 19 18 | 17 16 |
| -------- | -------------------- | ----- | ------- | ----- | ------ | ----- | ----- |
| x        | GEN                  |       |         | ZERO  |        |       |       |
| Bit 15   | 14 13 12             | 11 10 | 9 8     | 7 6   | 5 4    | 3 2   | 1 0   |
|          | ZERO                 |       |         |       | COB-ID |       |       |
| MSB      |                      |       |         |       |        |       | LSB   |
| Bit      | Notation             |       | Meaning |       |        |       |       |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |        |       |       |
| 11 to 29 | ZERO Set to "0"      |       |         |       |        |       |       |
0: The driver does not generate SYNC message. 
| 30  | GEN |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- |
1: The driver generate SYNC message.
| 31  | x Do not care. |     |     |     |     |     |     |
| --- | -------------- | --- | --- | --- | --- | --- | --- |
62

## Page 63

Object Dictionary
z  1006h: Communication cycle period
This object can be used to define the cycle period (in μs) for the SYNC interval.
Only multiples of 250 μs are permitted.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Communication cycle  |        |       | 0 to 1,000,000  |        |
| ----- | -------------------- | ------ | ----- | --------------- | ------ |
| 1006h | 00h                  | UINT32 | rw No |                 | μs Yes |
|       | period               |        |       | (0)             |        |
z  1008h: Manufacturer device name
This object contains the device name as character string.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Manufacturer device  |        |      | BLVD-KRD  |      |
| ----- | -------------------- | ------ | ---- | --------- | ---- |
| 1008h | 00h                  | STRING | c No |           | – No |
|       | name                 |        |      | BLVD-KBRD |      |
z  1009h: Manufacturer hardware version
This object contains the hardware version as character string.
“Rev.1.00” is indicated when the version is 1.00.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Manufacturer hardware  |        |      | Hardware  |      |
| ----- | ---------------------- | ------ | ---- | --------- | ---- |
| 1009h | 00h                    | STRING | c No |           | – No |
|       | version                |        |      | version   |      |
z  100Ah: Manufacturer software version
This object contains the software version as character string.
“V.1.00” is indicated when the version is 1.00.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Manufacturer software 
| 100Ah | 00h | STRING | c No | Software version | – No |
| ----- | --- | ------ | ---- | ---------------- | ---- |
version
z  100Ch: Guard time
The objects at “Guard time (100Ch)” and “Life time factor (100Dh)” indicate the configured guard time respectively the 
life time factor. The life time factor multiplied with the guard time provides the life time for the node guarding 
protocol. The value is specified in milliseconds.
If the value of the Object “Guard time” is set to “0”, then disable the node guarding.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 100Ch | 00h Guard time | UINT16 | rw No | 0 to 65,535 (0) | ms Yes |
| ----- | -------------- | ------ | ----- | --------------- | ------ |
z  100Dh: Life time factor
The life time factor multiplied with the guard time provides the life time for the node guarding protocol.
If the value of the Object “Life time factor” is set to “0”, then disable the node guarding.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 100Dh | 00h Life time factor | UINT8 | rw No | 0 to 255 (0) | – Yes |
| ----- | -------------------- | ----- | ----- | ------------ | ----- |
63

## Page 64

Object Dictionary
z  1010h: Store parameters
You can use this object to save the parameter settings in non-volatile memory.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Store parameters
Highest sub-index 
|     | 00h |     |     |     | UINT8 | ro  | No  | 2   | –   | No  |
| --- | --- | --- | --- | --- | ----- | --- | --- | --- | --- | --- |
supported
0000 0000h 
to 
|       | 01h | Save all parameters |     |     | UINT32 | rw  | No  |             | –   | No  |
| ----- | --- | ------------------- | --- | --- | ------ | --- | --- | ----------- | --- | --- |
| 1010h |     |                     |     |     |        |     |     | FFFF FFFFh  |     |     |
(0000 0000h)
0000 0000h 
|     |     | Save communication  |     |     |        |     |     | to          |     |     |
| --- | --- | ------------------- | --- | --- | ------ | --- | --- | ----------- | --- | --- |
|     | 02h |                     |     |     | UINT32 | rw  | No  |             | –   | No  |
|     |     | parameters          |     |     |        |     |     | FFFF FFFFh  |     |     |
(0000 0000h)
To prevent saving parameters by mistake, they are saved only when a specific signature is written to the appropriate 
sub-index. The signature is “save.”
| Signature |           | MSB |     |     | LSB |     |     |     |     |     |
| --------- | --------- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|           | character | e   | v   | a   | s   |     |     |     |     |     |
|           | hex       | 65h | 76h | 61h | 73h |     |     |     |     |     |
If you write “save” to sub-index 1, all parameters are saved.
If you write “save” to sub-index 2, the communications parameters (objects from 1000h to 1FFFh) are saved.
On reception of the correct signature in the appropriate sub-index the driver stores the parameter and then it 
confirms the SDO transmission (SDO download initiate response). 
If the storing failed, the driver responds with the SDO abort transfer service (abort code: 0606 0000h). 
If a wrong signature is written, the driver refuse to store and it responds with the SDO abort transfer service  
(abort code: 0800 002xh).
On read access to the appropriate sub-index the driver provides information about its storage functionality with the 
following format.
| Bit 31 | 30 29 | 28  | 27  | 26 25 | 24  | 23 22 | 21 20 | 19 18 | 17  | 16  |
| ------ | ----- | --- | --- | ----- | --- | ----- | ----- | ----- | --- | --- |
RSV [16]
| Bit 15  | 14 13    | 12                                                | 11  | 10      | 9 8      | 7 6 | 5 4 | 3 2 | 1    | 0   |
| ------- | -------- | ------------------------------------------------- | --- | ------- | -------- | --- | --- | --- | ---- | --- |
|         |          |                                                   |     |         | RSV [14] |     |     |     | AUTO | CMD |
| MSB     |          |                                                   |     |         |          |     |     |     |      | LSB |
| Bit     | Notation |                                                   |     | Meaning |          |     |     |     |      |     |
| 0       | CMD      | Always 1: The driver saves parameters on command. |     |         |          |     |     |     |      |     |
| 1       | AUTO     | Always 0                                          |     |         |          |     |     |     |      |     |
| 2 to 31 | RSV      | Reserved                                          |     |         |          |     |     |     |      |     |
Note: Autonomous saving means that a driver stores the storable parameters in a non-volatile memory without user 
request.
64

## Page 65

Object Dictionary
z  1011h: Restore default parameters
You can use this object to restore the parameters to the default values.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Restore default parameters
Highest sub-index 
|     | 00h |     |     |     | UINT8 | ro  | No  | 2   | –   | No  |
| --- | --- | --- | --- | --- | ----- | --- | --- | --- | --- | --- |
supported
0000 0000h 
|       |     | Restore all default  |     |     |        |     |     | to          |     |     |
| ----- | --- | -------------------- | --- | --- | ------ | --- | --- | ----------- | --- | --- |
|       | 01h |                      |     |     | UINT32 | rw  | No  |             | –   | No  |
| 1011h |     | parameters           |     |     |        |     |     | FFFF FFFFh  |     |     |
(0000 0000h)
0000 0000h 
|     |     | Restore communication  |     |     |        |     |     | to          |     |     |
| --- | --- | ---------------------- | --- | --- | ------ | --- | --- | ----------- | --- | --- |
|     | 02h |                        |     |     | UINT32 | rw  | No  |             | –   | No  |
|     |     | default parameters     |     |     |        |     |     | FFFF FFFFh  |     |     |
(0000 0000h)
To prevent restoring the parameters to the default values by mistake, the parameters are restored to the default 
values only when a specific signature is written to the appropriate sub-index. The signature is “load.”
| Signature |           | MSB |     |     | LSB |     |     |     |     |     |
| --------- | --------- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|           | character | d   | a   | o   | l   |     |     |     |     |     |
|           | hex       | 64h | 61h | 6Fh | 6Ch |     |     |     |     |     |
If you write “load” to sub-index 1, all parameters are restored to the default values.
If you write “load” to sub-index 2, the communications parameters (objects from 1000h to 1FFFh) are restored to the 
default values.
On reception of the correct signature in the appropriate sub-index the driver restores the default parameters and then 
it confirms the SDO transmission (SDO download initiate response). 
If the restoring failed, the driver responds with the SDO abort transfer service (abort code: 0606 0000h). 
If a wrong signature is written, the driver refuses to restore the defaults and responds with the SDO abort transfer 
service (abort code: 0800 002xh).
The default values will be set valid after the driver is reset (NMT service reset node for sub-index from 01h to 7Fh, NMT 
service reset communication for sub-index 02h) or power cycled.
On read access to the appropriate sub-index the driver provides information about its default parameter restoring 
capability with the following format.
| Bit 31 | 30 29 | 28  | 27  | 26 25 | 24  | 23 22 | 21  | 20 19 18 | 17  | 16  |
| ------ | ----- | --- | --- | ----- | --- | ----- | --- | -------- | --- | --- |
RSV [16]
| Bit 15  | 14 13    | 12                                        | 11  | 10      | 9 8      | 7 6 | 5   | 4 3 2 | 1   | 0   |
| ------- | -------- | ----------------------------------------- | --- | ------- | -------- | --- | --- | ----- | --- | --- |
|         |          |                                           |     |         | RSV [15] |     |     |       |     | CMD |
| MSB     |          |                                           |     |         |          |     |     |       |     | LSB |
| Bit     | Notation |                                           |     | Meaning |          |     |     |       |     |     |
| 0       | CMD      | Always 1: The driver restores parameters. |     |         |          |     |     |       |     |     |
| 1 to 31 | RSV      | Reserved                                  |     |         |          |     |     |       |     |     |
65

## Page 66

Object Dictionary
z  1014h: COB-ID EMCY
This object can be used to define the COB-ID for the Emergency message.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 1014h | 00h | COB-ID EMCY |     | UINT32 |     | rw No | 80h + Node-ID |     | –   | Yes |
| ----- | --- | ----------- | --- | ------ | --- | ----- | ------------- | --- | --- | --- |
Data Description
| Bit 31   | 30 29                  | 28 27 | 26      | 25  | 24 23 | 22 21         | 20  | 19  | 18 17 | 16  |
| -------- | ---------------------- | ----- | ------- | --- | ----- | ------------- | --- | --- | ----- | --- |
| VALID    |                        |       |         |     | ZERO  |               |     |     |       |     |
| Bit 15   | 14 13                  | 12 11 | 10      | 9   | 8 7   | 6 5           | 4   | 3   | 2 1   | 0   |
|          | ZERO                   |       |         |     |       | 11bit-Node-ID |     |     |       |     |
| MSB      |                        |       |         |     |       |               |     |     |       | LSB |
| Bit      | Notation               |       | Meaning |     |       |               |     |     |       |     |
| 0 to 10  | Node-ID 11-bit Node-ID |       |         |     |       |               |     |     |       |     |
| 11 to 30 | ZERO Set to "0"        |       |         |     |       |               |     |     |       |     |
0: EMCY exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: EMCY does not exist / is not valid
z  1016h: Consumer heartbeat time
This object defines the cycle time of the Consumer Heartbeat of the Network Management CANopen service and the 
Node-ID of the Producer of the Heartbeat.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Consumer heartbeat time
Highest sub-index 
|     | 00h |     |     | UINT8 |     | ro No |     | 1   | –   | No  |
| --- | --- | --- | --- | ----- | --- | ----- | --- | --- | --- | --- |
supported
1016h
0000 0000h 
|     |     | Consumer heartbeat  |     |        |     |       |     | to          |     |     |
| --- | --- | ------------------- | --- | ------ | --- | ----- | --- | ----------- | --- | --- |
|     | 01h |                     |     | UINT32 |     | rw No |     |             | –   | Yes |
|     |     | time                |     |        |     |       |     | 00FF FFFFh  |     |     |
(0000 0000h)
Data Description of Sub-index 01h
| Bit 31 | 30 29 | 28 27 | 26 25 | 24  | 23  | 22 21 | 20      | 19  | 18 17 | 16  |
| ------ | ----- | ----- | ----- | --- | --- | ----- | ------- | --- | ----- | --- |
|        |       | ZERO  |       |     |     |       | Node-ID |     |       |     |
| Bit 15 | 14 13 | 12 11 | 10    | 9 8 | 7   | 6 5   | 4       | 3   | 2 1   | 0   |
Heartbeat time
| MSB     |                |                                                    |     |     |     |         |     |     |     | LSB |
| ------- | -------------- | -------------------------------------------------- | --- | --- | --- | ------- | --- | --- | --- | --- |
| Bit     | Notation       |                                                    |     |     |     | Meaning |     |     |     |     |
| 0 to 15 | Heartbeat time | The time of the Consumer Heartbeat in millisecond. |     |     |     |         |     |     |     |     |
16 to 23 Node-ID The Node-ID of the Producer whose Heartbeat is to be monitored.
| 24 to 31 | ZERO | Set to "0" |     |     |     |     |     |     |     |     |
| -------- | ---- | ---------- | --- | --- | --- | --- | --- | --- | --- | --- |
66

## Page 67

Object Dictionary
z  1017h: Producer heartbeat time
This object defines the cycle time of the Heartbeat of the Network Management CANopen service in milliseconds.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Producer heartbeat 
| 1017h | 00h | UINT16 | rw No | 0 to 65,535 (0) | ms Yes |
| ----- | --- | ------ | ----- | --------------- | ------ |
time
z  1018h: Identity object
This object contains general information on the driver.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Identity object
Highest sub-index 
|     | 00h | UINT8 | ro No | 2   | – No |
| --- | --- | ----- | ----- | --- | ---- |
supported
|     | 01h Vender ID | UINT32 | ro No | 0000 02BEh | – No |
| --- | ------------- | ------ | ----- | ---------- | ---- |
1018h
BLVD-KRD 
0000 13F7h
|     | 02h Product code | UINT32 | ro No |     | – No |
| --- | ---------------- | ------ | ----- | --- | ---- |
BLVD-KBRD 
0000 1430h
z  1200h: SDO server parameter
This object is the COB-ID for SDO server.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
SDO server parameter
Highest sub-index 
|     | 00h | UINT8 | ro No | 2   | – No |
| --- | --- | ----- | ----- | --- | ---- |
supported
| 1200h | COB-ID client ->   |        |       |                |      |
| ----- | ------------------ | ------ | ----- | -------------- | ---- |
|       | 01h                | UINT32 | ro No | 600h + Node-ID | – No |
server (rx)
COB-ID server ->  
|     | 02h | UINT32 | ro No | 580h + Node-ID | – No |
| --- | --- | ------ | ----- | -------------- | ---- |
client (tx)
67

## Page 68

Object Dictionary
z  1400h: 1st RPDO communication parameter
This object contains the communication parameters for the PDOs that the driver is able to receive. (RPDO1)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1st RPDO communication parameter
Highest sub-index 
|     | 00h |     |     | UINT8 | ro No | 2   | –   | No  |
| --- | --- | --- | --- | ----- | ----- | --- | --- | --- |
supported
0000 0000h 
1400h
to 
|     | 01h | COB-ID used by RPDO |     | UINT32 | rw No |     | –   | Yes |
| --- | --- | ------------------- | --- | ------ | ----- | --- | --- | --- |
FFFF FFFFh 
(200h + Node-ID)
|     | 02h | Transmission type |     | UINT8 | rw No | 0 to 255 (255) | –   | Yes |
| --- | --- | ----------------- | --- | ----- | ----- | -------------- | --- | --- |
Data Description of Sub-index 01h
| Bit 31   | 30 29                | 28 27 | 26      | 25 24 | 23 22 21 | 20 19 | 18 17 | 16  |
| -------- | -------------------- | ----- | ------- | ----- | -------- | ----- | ----- | --- |
| VALID    |                      |       |         | ZERO  |          |       |       |     |
| Bit 15   | 14 13                | 12 11 | 10      | 9 8   | 7 6 5    | 4 3   | 2 1   | 0   |
|          | ZERO                 |       |         |       | COB-ID   |       |       |     |
| MSB      |                      |       |         |       |          |       |       | LSB |
| Bit      | Notation             |       | Meaning |       |          |       |       |     |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |          |       |       |     |
| 11 to 30 | ZERO Set to "0"      |       |         |       |          |       |       |     |
0: PDO exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- |
1: PDO does not exist / is not valid
Data Description of Sub-index 02h
| Value |     |     |     | Description |     |     |     |     |
| ----- | --- | --- | --- | ----------- | --- | --- | --- | --- |
Synchronous 
00h If the transmission type of RPDO is set to synchronous, the received PDO data will be pending then updated 
on the next reception of the SYNC object.
event-driven (manufacturer-specific) 
If the transmission type of RPDO is set to event-drivent (manufacturer-specific), the received PDO data will 
be updated immediately. 
FEh
The driver executes the following contents when received data in this transmission type. 
- The driver lssue RTR to the corresponding TPDO 
- The driver reset the node lifetime
event-driven 
FFh
If the transmission type of RPDO is set to event-driven, the received PDO data will be updated immediately.
Sub-index 02h defines the reception character of the RPDO. An attempt to change the value of the transmission type 
to any not supported value will be responded with the SDO abort transfer service (abort code: 0609 0030h).
68

## Page 69

Object Dictionary
z  1401h: 2nd RPDO communication parameter
This object contains the communication parameters for the PDOs that the driver is able to receive. (RPDO2)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
2nd RPDO communication parameter
Highest sub-index 
|     | 00h |     |     | UINT8 | ro No | 2   | –   | No  |
| --- | --- | --- | --- | ----- | ----- | --- | --- | --- |
supported
0000 0000h 
1401h
to 
|     | 01h | COB-ID used by RPDO |     | UINT32 | rw No |     | –   | Yes |
| --- | --- | ------------------- | --- | ------ | ----- | --- | --- | --- |
FFFF FFFFh 
(300h + Node-ID)
|     | 02h | Transmission type |     | UINT8 | rw No | 0 to 255 (255) | –   | Yes |
| --- | --- | ----------------- | --- | ----- | ----- | -------------- | --- | --- |
Data Description of Sub-index 01h
| Bit 31   | 30 29                | 28 27 | 26      | 25 24 | 23 22 21 | 20 19 | 18 17 | 16  |
| -------- | -------------------- | ----- | ------- | ----- | -------- | ----- | ----- | --- |
| VALID    |                      |       |         | ZERO  |          |       |       |     |
| Bit 15   | 14 13                | 12 11 | 10      | 9 8   | 7 6 5    | 4 3   | 2 1   | 0   |
|          | ZERO                 |       |         |       | COB-ID   |       |       |     |
| MSB      |                      |       |         |       |          |       |       | LSB |
| Bit      | Notation             |       | Meaning |       |          |       |       |     |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |          |       |       |     |
| 11 to 30 | ZERO Set to "0"      |       |         |       |          |       |       |     |
0: PDO exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- |
1: PDO does not exist / is not valid
Data Description of Sub-index 02h
| Value |     |     |     | Description |     |     |     |     |
| ----- | --- | --- | --- | ----------- | --- | --- | --- | --- |
Synchronous 
00h If the transmission type of RPDO is set to synchronous, the received PDO data will be pending then updated 
on the next reception of the SYNC object.
event-driven (manufacturer-specific) 
If the transmission type of RPDO is set to event-drivent (manufacturer-specific), the received PDO data will 
be updated immediately. 
FEh
The driver executes the following contents when received data in this transmission type. 
- The driver lssue RTR to the corresponding TPDO 
- The driver reset the node lifetime
event-driven 
FFh
If the transmission type of RPDO is set to event-driven, the received PDO data will be updated immediately.
Sub-index 02h defines the reception character of the RPDO. An attempt to change the value of the transmission type 
to any not supported value will be responded with the SDO abort transfer service (abort code: 0609 0030h).
69

## Page 70

Object Dictionary
z  1402h: 3rd RPDO communication parameter
This object contains the communication parameters for the PDOs that the driver is able to receive. (RPDO3)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
3rd RPDO communication parameter
Highest sub-index 
|     | 00h |     |     | UINT8 | ro No | 2   | –   | No  |
| --- | --- | --- | --- | ----- | ----- | --- | --- | --- |
supported
0000 0000h 
1402h
to 
|     | 01h | COB-ID used by RPDO |     | UINT32 | rw No |     | –   | Yes |
| --- | --- | ------------------- | --- | ------ | ----- | --- | --- | --- |
FFFF FFFFh 
(400h + Node-ID)
|     | 02h | Transmission type |     | UINT8 | rw No | 0 to 255 (255) | –   | Yes |
| --- | --- | ----------------- | --- | ----- | ----- | -------------- | --- | --- |
Data Description of Sub-index 01h
| Bit 31   | 30 29                | 28 27 | 26      | 25 24 | 23 22 21 | 20 19 | 18 17 | 16  |
| -------- | -------------------- | ----- | ------- | ----- | -------- | ----- | ----- | --- |
| VALID    |                      |       |         | ZERO  |          |       |       |     |
| Bit 15   | 14 13                | 12 11 | 10      | 9 8   | 7 6 5    | 4 3   | 2 1   | 0   |
|          | ZERO                 |       |         |       | COB-ID   |       |       |     |
| MSB      |                      |       |         |       |          |       |       | LSB |
| Bit      | Notation             |       | Meaning |       |          |       |       |     |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |          |       |       |     |
| 11 to 30 | ZERO Set to "0"      |       |         |       |          |       |       |     |
0: PDO exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- |
1: PDO does not exist / is not valid
Data Description of Sub-index 02h
| Value |     |     |     | Description |     |     |     |     |
| ----- | --- | --- | --- | ----------- | --- | --- | --- | --- |
Synchronous 
00h If the transmission type of RPDO is set to synchronous, the received PDO data will be pending then updated 
on the next reception of the SYNC object.
event-driven (manufacturer-specific) 
If the transmission type of RPDO is set to event-drivent (manufacturer-specific), the received PDO data will 
be updated immediately. 
FEh
The driver executes the following contents when received data in this transmission type. 
- The driver lssue RTR to the corresponding TPDO 
- The driver reset the node lifetime
event-driven 
FFh
If the transmission type of RPDO is set to event-driven, the received PDO data will be updated immediately.
Sub-index 02h defines the reception character of the RPDO. An attempt to change the value of the transmission type 
to any not supported value will be responded with the SDO abort transfer service (abort code: 0609 0030h).
70

## Page 71

Object Dictionary
z  1403h: 4th RPDO communication parameter
This object contains the communication parameters for the PDOs that the driver is able to receive. (RPDO4)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
4th RPDO communication parameter
Highest sub-index 
|     | 00h |     |     | UINT8 | ro No | 2   | –   | No  |
| --- | --- | --- | --- | ----- | ----- | --- | --- | --- |
supported
0000 0000h 
1403h
to 
|     | 01h | COB-ID used by RPDO |     | UINT32 | rw No |     | –   | Yes |
| --- | --- | ------------------- | --- | ------ | ----- | --- | --- | --- |
FFFF FFFFh 
(500h + Node-ID)
|     | 02h | Transmission type |     | UINT8 | rw No | 0 to 255 (255) | –   | Yes |
| --- | --- | ----------------- | --- | ----- | ----- | -------------- | --- | --- |
Data Description of Sub-index 01h
| Bit 31   | 30 29                | 28 27 | 26      | 25 24 | 23 22 21 | 20 19 | 18 17 | 16  |
| -------- | -------------------- | ----- | ------- | ----- | -------- | ----- | ----- | --- |
| VALID    |                      |       |         | ZERO  |          |       |       |     |
| Bit 15   | 14 13                | 12 11 | 10      | 9 8   | 7 6 5    | 4 3   | 2 1   | 0   |
|          | ZERO                 |       |         |       | COB-ID   |       |       |     |
| MSB      |                      |       |         |       |          |       |       | LSB |
| Bit      | Notation             |       | Meaning |       |          |       |       |     |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |          |       |       |     |
| 11 to 30 | ZERO Set to "0"      |       |         |       |          |       |       |     |
0: PDO exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- |
1: PDO does not exist / is not valid
Data Description of Sub-index 02h
| Value |     |     |     | Description |     |     |     |     |
| ----- | --- | --- | --- | ----------- | --- | --- | --- | --- |
Synchronous 
00h If the transmission type of RPDO is set to synchronous, the received PDO data will be pending then updated 
on the next reception of the SYNC object.
event-driven (manufacturer-specific) 
If the transmission type of RPDO is set to event-drivent (manufacturer-specific), the received PDO data will 
be updated immediately. 
FEh
The driver executes the following contents when received data in this transmission type. 
- The driver lssue RTR to the corresponding TPDO 
- The driver reset the node lifetime
event-driven 
FFh
If the transmission type of RPDO is set to event-driven, the received PDO data will be updated immediately.
Sub-index 02h defines the reception character of the RPDO. An attempt to change the value of the transmission type 
to any not supported value will be responded with the SDO abort transfer service (abort code: 0609 0030h).
71

## Page 72

Object Dictionary
z  1600h: 1st RPDO mapping parameter
This object contains the mapping parameters for the PDOs the driver is able to receive. (RPDO1)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1st RPDO mapping parameter
Number of mapped 
|     | 00h application objects in  |     | UINT8 | rw  | No  | 1   | – Yes |
| --- | --------------------------- | --- | ----- | --- | --- | --- | ----- |
PDO
0000 0000h 
to 
|     | 01h 1st application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6040 0010h)
0000 0000h 
to 
1600h
|     | 02h 2nd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 03h 3rd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 04h 4th application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
Data Description of Sub-index 01h to 04h
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Index [16]
| Bit 15 | 14 13 12      | 11 10 | 9 8 | 7 6     | 5 4        | 3 2 | 1 0 |
| ------ | ------------- | ----- | --- | ------- | ---------- | --- | --- |
|        | Sub-index [8] |       |     |         | Length [8] |     |     |
| MSB    |               |       |     |         |            |     | LSB |
| Bit    | Notation      |       |     | Meaning |            |     |     |
0 to 7 Length This contains the length of the object to be mapped in units of bits.
| 8 to 15  | Sub-index This contains the sub-index of the object to be mapped. |     |     |     |     |     |     |
| -------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 16 to 31 | Index This contains the index of the object to be mapped.         |     |     |     |     |     |     |
72

## Page 73

Object Dictionary
z  1601h: 2nd RPDO mapping parameter
This object contains the mapping parameters for the PDOs the driver is able to receive. (RPDO2)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
2nd RPDO mapping parameter
Number of mapped 
|     | 00h application objects in  |     | UINT8 | rw  | No  | 2   | – Yes |
| --- | --------------------------- | --- | ----- | --- | --- | --- | ----- |
PDO
0000 0000h 
to 
|     | 01h 1st application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6040 0010h)
0000 0000h 
to 
1601h
|     | 02h 2nd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6060 0008h)
0000 0000h 
to 
|     | 03h 3rd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 04h 4th application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
Data Description of Sub-index 01h to 04h
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Index [16]
| Bit 15 | 14 13 12      | 11 10 | 9 8 | 7 6     | 5 4        | 3 2 | 1 0 |
| ------ | ------------- | ----- | --- | ------- | ---------- | --- | --- |
|        | Sub-index [8] |       |     |         | Length [8] |     |     |
| MSB    |               |       |     |         |            |     | LSB |
| Bit    | Notation      |       |     | Meaning |            |     |     |
0 to 7 Length This contains the length of the object to be mapped in units of bits.
| 8 to 15  | Sub-index This contains the sub-index of the object to be mapped. |     |     |     |     |     |     |
| -------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 16 to 31 | Index This contains the index of the object to be mapped.         |     |     |     |     |     |     |
73

## Page 74

Object Dictionary
z  1602h: 3rd RPDO mapping parameter
This object contains the mapping parameters for the PDOs the driver is able to receive. (RPDO3)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
3rd RPDO mapping parameter
Number of mapped 
|     | 00h application objects in  |     | UINT8 | rw  | No  | 2   | – Yes |
| --- | --------------------------- | --- | ----- | --- | --- | --- | ----- |
PDO
0000 0000h 
to 
|     | 01h 1st application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6040 0010h)
0000 0000h 
to 
1602h
|     | 02h 2nd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(607A 0020h)
0000 0000h 
to 
|     | 03h 3rd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 04h 4th application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
Data Description of Sub-index 01h to 04h
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Index [16]
| Bit 15 | 14 13 12      | 11 10 | 9 8 | 7 6     | 5 4        | 3 2 | 1 0 |
| ------ | ------------- | ----- | --- | ------- | ---------- | --- | --- |
|        | Sub-index [8] |       |     |         | Length [8] |     |     |
| MSB    |               |       |     |         |            |     | LSB |
| Bit    | Notation      |       |     | Meaning |            |     |     |
0 to 7 Length This contains the length of the object to be mapped in units of bits.
| 8 to 15  | Sub-index This contains the sub-index of the object to be mapped. |     |     |     |     |     |     |
| -------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 16 to 31 | Index This contains the index of the object to be mapped.         |     |     |     |     |     |     |
74

## Page 75

Object Dictionary
z  1603h: 4th RPDO mapping parameter
This object contains the mapping parameters for the PDOs the driver is able to receive. (RPDO4)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
4th RPDO mapping parameter
Number of mapped 
|     | 00h application objects in  |     | UINT8 | rw  | No  | 2   | – Yes |
| --- | --------------------------- | --- | ----- | --- | --- | --- | ----- |
PDO
0000 0000h 
to 
|     | 01h 1st application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6040 0010h)
0000 0000h 
to 
1603h
|     | 02h 2nd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(60FF 0020h)
0000 0000h 
to 
|     | 03h 3rd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 04h 4th application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
Data Description of Sub-index 01h to 04h
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Index [16]
| Bit 15 | 14 13 12      | 11 10 | 9 8 | 7 6     | 5 4        | 3 2 | 1 0 |
| ------ | ------------- | ----- | --- | ------- | ---------- | --- | --- |
|        | Sub-index [8] |       |     |         | Length [8] |     |     |
| MSB    |               |       |     |         |            |     | LSB |
| Bit    | Notation      |       |     | Meaning |            |     |     |
0 to 7 Length This contains the length of the object to be mapped in units of bits.
| 8 to 15  | Sub-index This contains the sub-index of the object to be mapped. |     |     |     |     |     |     |
| -------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 16 to 31 | Index This contains the index of the object to be mapped.         |     |     |     |     |     |     |
75

## Page 76

Object Dictionary
z  1800h: 1st TPDO communication parameter
This object contains the communication parameters for the PDOs the driver is able to transmit. (TPDO1)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1st TPDO communication parameter
Highest sub-index 
|     | 00h |     |     | UINT8 | ro No | 5   | –   | No  |
| --- | --- | --- | --- | ----- | ----- | --- | --- | --- |
supported
0000 0000h 
to 
|       | 01h | COB-ID used by TPDO |     | UINT32 | rw No | FFFF FFFFh   | –   | Yes |
| ----- | --- | ------------------- | --- | ------ | ----- | ------------ | --- | --- |
| 1800h |     |                     |     |        |       | (4000 0180h  |     |     |
 + Node-ID)
|     | 02h | Transmission type |     | UINT8  | rw No | 0 to 255 (255)    | –      | Yes |
| --- | --- | ----------------- | --- | ------ | ----- | ----------------- | ------ | --- |
|     | 03h | Inhibit time      |     | UINT16 | rw No | 0 to 65,535 (50)  | 100 μs | Yes |
|     | 04h | Reserved          |     | –      | – –   | –                 | –      | –   |
|     | 05h | Event timer       |     | UINT16 | rw No | 0 to 65,535 (0)   | ms     | Yes |
Data Description of Sub-index 01h
| Bit 31   | 30 29                | 28 27 | 26      | 25 24 | 23 22 21 | 20 19 | 18 17 | 16  |
| -------- | -------------------- | ----- | ------- | ----- | -------- | ----- | ----- | --- |
| VALID    | RTR                  |       |         |       | ZERO     |       |       |     |
| Bit 15   | 14 13                | 12 11 | 10      | 9 8   | 7 6 5    | 4 3   | 2 1   | 0   |
|          | ZERO                 |       |         |       | COB-ID   |       |       |     |
| MSB      |                      |       |         |       |          |       |       | LSB |
| Bit      | Notation             |       | Meaning |       |          |       |       |     |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |          |       |       |     |
| 11 to 29 | ZERO Set to "0"      |       |         |       |          |       |       |     |
0: RTR allowed on this PDO 
| 30  | RTR |     |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: no RTR allowed on this PDO
0: PDO exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- |
1: PDO does not exist / is not valid
76

## Page 77

Object Dictionary
Sub-index 02h defines the transmission character of the TPDO. An attempt to change the value of the transmission
type to any not supported value is responded with the SDO abort transfer service (abort code: 0609 0030h).
Data Description of Sub-index 02h
Value Description
00h synchronous (acyclic)
01h synchronous (cyclic every SYNC)
02h synchronous (cyclic every 2nd SYNC)
03h synchronous (cyclic every 3rd SYNC)
: :
: :
F0h synchronous (cyclic every 240th SYNC)
F1h to FBh Reserved
FCh RTR-only (synchronous)
FDh RTR-only (event-driven)
FEh event-driven
FFh event-driven
Sub-index 03h contains the inhibit time. The time is the minimum interval for PDO transmission if the transmission
type is set to FEh and FFh. The value is defined as multiple of 100 microseconds.
The value of 0 is disable the inhibit time.
The value shall not be changed while the PDO exists (bit 31 of sub-index 01h is set to 0b).
Sub-index 04h is reserved. It does shall not be implemented; in this case read or write access leads to the SDO abort
transfer service (abort code: 0609 0011h).
Sub-index 05h contains the event-timer. The time is the maximum interval for PDO transmission if the transmission
type is set to FEh and FFh.
The value is defined as multiple of 1 millisecond. The value of 0 is disable the event-timer.
77

## Page 78

Object Dictionary
z  1801h: 2nd TPDO communication parameter
This object contains the communication parameters for the PDOs the driver is able to transmit. (TPDO2)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
2nd TPDO communication parameter
Highest sub-index 
|     | 00h |     |     | UINT8 | ro No | 5   | –   | No  |
| --- | --- | --- | --- | ----- | ----- | --- | --- | --- |
supported
0000 0000h 
to 
|       | 01h | COB-ID used by TPDO |     | UINT32 | rw No | FFFF FFFFh   | –   | Yes |
| ----- | --- | ------------------- | --- | ------ | ----- | ------------ | --- | --- |
| 1801h |     |                     |     |        |       | (4000 0280h  |     |     |
 + Node-ID)
|     | 02h | Transmission type |     | UINT8  | rw No | 0 to 255 (255)    | –      | Yes |
| --- | --- | ----------------- | --- | ------ | ----- | ----------------- | ------ | --- |
|     | 03h | Inhibit time      |     | UINT16 | rw No | 0 to 65,535 (50)  | 100 μs | Yes |
|     | 04h | Reserved          |     | –      | – –   | –                 | –      | –   |
|     | 05h | Event timer       |     | UINT16 | rw No | 0 to 65,535 (0)   | ms     | Yes |
Data Description of Sub-index 01h
| Bit 31   | 30 29                | 28 27 | 26      | 25 24 | 23 22 21 | 20 19 | 18 17 | 16  |
| -------- | -------------------- | ----- | ------- | ----- | -------- | ----- | ----- | --- |
| VALID    | RTR                  |       |         |       | ZERO     |       |       |     |
| Bit 15   | 14 13                | 12 11 | 10      | 9 8   | 7 6 5    | 4 3   | 2 1   | 0   |
|          | ZERO                 |       |         |       | COB-ID   |       |       |     |
| MSB      |                      |       |         |       |          |       |       | LSB |
| Bit      | Notation             |       | Meaning |       |          |       |       |     |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |          |       |       |     |
| 11 to 29 | ZERO Set to "0"      |       |         |       |          |       |       |     |
0: RTR allowed on this PDO 
| 30  | RTR |     |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: no RTR allowed on this PDO
0: PDO exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- |
1: PDO does not exist / is not valid
78

## Page 79

Object Dictionary
Sub-index 02h defines the transmission character of the TPDO. An attempt to change the value of the transmission
type to any not supported value is responded with the SDO abort transfer service (abort code: 0609 0030h).
Data Description of Sub-index 02h
Value Description
00h synchronous (acyclic)
01h synchronous (cyclic every SYNC)
02h synchronous (cyclic every 2nd SYNC)
03h synchronous (cyclic every 3rd SYNC)
: :
: :
F0h synchronous (cyclic every 240th SYNC)
F1h to FBh Reserved
FCh RTR-only (synchronous)
FDh RTR-only (event-driven)
FEh event-driven
FFh event-driven
Sub-index 03h contains the inhibit time. The time is the minimum interval for PDO transmission if the transmission
type is set to FEh and FFh. The value is defined as multiple of 100 microseconds.
The value of 0 is disable the inhibit time.
The value shall not be changed while the PDO exists (bit 31 of sub-index 01h is set to 0b).
Sub-index 04h is reserved. It does shall not be implemented; in this case read or write access leads to the SDO abort
transfer service (abort code: 0609 0011h).
Sub-index 05h contains the event-timer. The time is the maximum interval for PDO transmission if the transmission
type is set to FEh and FFh.
The value is defined as multiple of 1 millisecond. The value of 0 is disable the event-timer.
79

## Page 80

Object Dictionary
z  1802h: 3rd TPDO communication parameter
This object contains the communication parameters for the PDOs the driver is able to transmit (TPDO3).
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
3rd TPDO communication parameter
Highest sub-index 
|     | 00h |     |     | UINT8 | ro No | 5   | –   | No  |
| --- | --- | --- | --- | ----- | ----- | --- | --- | --- |
supported
0000 0000h 
to 
|       | 01h | COB-ID used by TPDO |     | UINT32 | rw No | FFFF FFFFh   | –   | Yes |
| ----- | --- | ------------------- | --- | ------ | ----- | ------------ | --- | --- |
| 1802h |     |                     |     |        |       | (4000 0380h  |     |     |
 + Node-ID)
|     | 02h | Transmission type |     | UINT8  | rw No | 0 to 255 (1)      | –      | Yes |
| --- | --- | ----------------- | --- | ------ | ----- | ----------------- | ------ | --- |
|     | 03h | Inhibit time      |     | UINT16 | rw No | 0 to 65,535 (50)  | 100 μs | Yes |
|     | 04h | Reserved          |     | –      | – –   | –                 | –      | –   |
|     | 05h | Event timer       |     | UINT16 | rw No | 0 to 65,535 (0)   | ms     | Yes |
Data Description of Sub-index 01h
| Bit 31   | 30 29                | 28 27 | 26      | 25 24 | 23 22 21 | 20 19 | 18 17 | 16  |
| -------- | -------------------- | ----- | ------- | ----- | -------- | ----- | ----- | --- |
| VALID    | RTR                  |       |         |       | ZERO     |       |       |     |
| Bit 15   | 14 13                | 12 11 | 10      | 9 8   | 7 6 5    | 4 3   | 2 1   | 0   |
|          | ZERO                 |       |         |       | COB-ID   |       |       |     |
| MSB      |                      |       |         |       |          |       |       | LSB |
| Bit      | Notation             |       | Meaning |       |          |       |       |     |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |          |       |       |     |
| 11 to 29 | ZERO Set to "0"      |       |         |       |          |       |       |     |
0: RTR allowed on this PDO 
| 30  | RTR |     |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: no RTR allowed on this PDO
0: PDO exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- |
1: PDO does not exist / is not valid
80

## Page 81

Object Dictionary
Sub-index 02h defines the transmission character of the TPDO. An attempt to change the value of the transmission
type to any not supported value is responded with the SDO abort transfer service (abort code: 0609 0030h).
Data Description of Sub-index 02h
Value Description
00h synchronous (acyclic)
01h synchronous (cyclic every SYNC)
02h synchronous (cyclic every 2nd SYNC)
03h synchronous (cyclic every 3rd SYNC)
: :
: :
F0h synchronous (cyclic every 240th SYNC)
F1h to FBh Reserved
FCh RTR-only (synchronous)
FDh RTR-only (event-driven)
FEh event-driven
FFh event-driven
Sub-index 03h contains the inhibit time. The time is the minimum interval for PDO transmission if the transmission
type is set to FEh and FFh. The value is defined as multiple of 100 microseconds.
The value of 0 is disable the inhibit time.
The value shall not be changed while the PDO exists (bit 31 of sub-index 01h is set to 0b).
Sub-index 04h is reserved. It does shall not be implemented; in this case read or write access leads to the SDO abort
transfer service (abort code: 0609 0011h).
Sub-index 05h contains the event-timer. The time is the maximum interval for PDO transmission if the transmission
type is set to FEh and FFh.
The value is defined as multiple of 1 millisecond. The value of 0 is disable the event-timer.
81

## Page 82

Object Dictionary
z  1803h: 4th TPDO communication parameter
This object contains the communication parameters for the PDOs the driver is able to transmit (TPDO4).
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
4th TPDO communication parameter
Highest sub-index 
|     | 00h |     |     | UINT8 | ro No | 5   | –   | No  |
| --- | --- | --- | --- | ----- | ----- | --- | --- | --- |
supported
0000 0000h 
to 
|       | 01h | COB-ID used by TPDO |     | UINT32 | rw No | FFFF FFFFh   | –   | Yes |
| ----- | --- | ------------------- | --- | ------ | ----- | ------------ | --- | --- |
| 1803h |     |                     |     |        |       | (4000 0480h  |     |     |
 + Node-ID)
|     | 02h | Transmission type |     | UINT8  | rw No | 0 to 255 (1)      | –      | Yes |
| --- | --- | ----------------- | --- | ------ | ----- | ----------------- | ------ | --- |
|     | 03h | Inhibit time      |     | UINT16 | rw No | 0 to 65,535 (50)  | 100 μs | Yes |
|     | 04h | Reserved          |     | –      | – –   | –                 | –      | –   |
|     | 05h | Event timer       |     | UINT16 | rw No | 0 to 65,535 (0)   | ms     | Yes |
Data Description of Sub-index 01h
| Bit 31   | 30 29                | 28 27 | 26      | 25 24 | 23 22 21 | 20 19 | 18 17 | 16  |
| -------- | -------------------- | ----- | ------- | ----- | -------- | ----- | ----- | --- |
| VALID    | RTR                  |       |         |       | ZERO     |       |       |     |
| Bit 15   | 14 13                | 12 11 | 10      | 9 8   | 7 6 5    | 4 3   | 2 1   | 0   |
|          | ZERO                 |       |         |       | COB-ID   |       |       |     |
| MSB      |                      |       |         |       |          |       |       | LSB |
| Bit      | Notation             |       | Meaning |       |          |       |       |     |
| 0 to 10  | COB-ID 11-bit COB-ID |       |         |       |          |       |       |     |
| 11 to 29 | ZERO Set to "0"      |       |         |       |          |       |       |     |
0: RTR allowed on this PDO 
| 30  | RTR |     |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: no RTR allowed on this PDO
0: PDO exists / is valid 
| 31  | VALID |     |     |     |     |     |     |     |
| --- | ----- | --- | --- | --- | --- | --- | --- | --- |
1: PDO does not exist / is not valid
82

## Page 83

Object Dictionary
Sub-index 02h defines the transmission character of the TPDO. An attempt to change the value of the transmission
type to any not supported value is responded with the SDO abort transfer service (abort code: 0609 0030h).
Data Description of Sub-index 02h
Value Description
00h synchronous (acyclic)
01h synchronous (cyclic every SYNC)
02h synchronous (cyclic every 2nd SYNC)
03h synchronous (cyclic every 3rd SYNC)
: :
: :
F0h synchronous (cyclic every 240th SYNC)
F1h to FBh Reserved
FCh RTR-only (synchronous)
FDh RTR-only (event-driven)
FEh event-driven
FFh event-driven
Sub-index 03h contains the inhibit time. The time is the minimum interval for PDO transmission if the transmission
type is set to FEh and FFh. The value is defined as multiple of 100 microseconds.
The value of 0 is disable the inhibit time.
The value shall not be changed while the PDO exists (bit 31 of sub-index 01h is set to 0b).
Sub-index 04h is reserved. It does shall not be implemented; in this case read or write access leads to the SDO abort
transfer service (abort code: 0609 0011h).
Sub-index 05h contains the event-timer. The time is the maximum interval for PDO transmission if the transmission
type is set to FEh and FFh.
The value is defined as multiple of 1 millisecond. The value of 0 is disable the event-timer.
83

## Page 84

Object Dictionary
z  1A00h: 1st TPDO mapping parameter
This object contains the mapping for the PDOs the driver is able to transmit. (TPDO1)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1st TPDO mapping parameter
Number of mapped 
|     | 00h application objects in  |     | UINT8 | rw  | No  | 1   | – Yes |
| --- | --------------------------- | --- | ----- | --- | --- | --- | ----- |
TPDO
0000 0000h 
to 
|     | 01h 1st application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6041 0010h)
0000 0000h 
to 
1A00h
|     | 02h 2nd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 03h 3rd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 04h 4th application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
Data Description of Sub-index 01h to 04h
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Index [16]
| Bit 15 | 14 13 12      | 11 10 | 9 8 | 7 6     | 5 4        | 3 2 | 1 0 |
| ------ | ------------- | ----- | --- | ------- | ---------- | --- | --- |
|        | Sub-index [8] |       |     |         | Length [8] |     |     |
| MSB    |               |       |     |         |            |     | LSB |
| Bit    | Notation      |       |     | Meaning |            |     |     |
0 to 7 Length This contains the length of the object to be mapped in units of bits.
| 8 to 15  | Sub-index This contains the sub-index of the object to be mapped. |     |     |     |     |     |     |
| -------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 16 to 31 | Index This contains the index of the object to be mapped.         |     |     |     |     |     |     |
84

## Page 85

Object Dictionary
z  1A01h: 2nd TPDO mapping parameter
This object contains the mapping for the PDOs the driver is able to transmit. (TPDO2)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
2nd TPDO mapping parameter
Number of mapped 
|     | 00h application objects in  |     | UINT8 | rw  | No  | 2   | – Yes |
| --- | --------------------------- | --- | ----- | --- | --- | --- | ----- |
TPDO
0000 0000h 
to 
|     | 01h 1st application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6041 0010h)
0000 0000h 
to 
1A01h
|     | 02h 2nd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6061 0008h)
0000 0000h 
to 
|     | 03h 3rd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 04h 4th application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
Data Description of Sub-index 01h to 04h
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Index [16]
| Bit 15 | 14 13 12      | 11 10 | 9 8 | 7 6     | 5 4        | 3 2 | 1 0 |
| ------ | ------------- | ----- | --- | ------- | ---------- | --- | --- |
|        | Sub-index [8] |       |     |         | Length [8] |     |     |
| MSB    |               |       |     |         |            |     | LSB |
| Bit    | Notation      |       |     | Meaning |            |     |     |
0 to 7 Length This contains the length of the object to be mapped in units of bits.
| 8 to 15  | Sub-index This contains the sub-index of the object to be mapped. |     |     |     |     |     |     |
| -------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 16 to 31 | Index This contains the index of the object to be mapped.         |     |     |     |     |     |     |
85

## Page 86

Object Dictionary
z  1A02h: 3rd TPDO mapping parameter
This object contains the mapping for the PDOs the driver is able to transmit. (TPDO3)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
3rd TPDO mapping parameter
Number of mapped 
|     | 00h application objects in  |     | UINT8 | rw  | No  | 2   | – Yes |
| --- | --------------------------- | --- | ----- | --- | --- | --- | ----- |
TPDO
0000 0000h 
to 
|     | 01h 1st application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6041 0010h)
0000 0000h 
to 
1A02h
|     | 02h 2nd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6064 0020h)
0000 0000h 
to 
|     | 03h 3rd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 04h 4th application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
Data Description of Sub-index 01h to 04h
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Index [16]
| Bit 15 | 14 13 12      | 11 10 | 9 8 | 7 6     | 5 4        | 3 2 | 1 0 |
| ------ | ------------- | ----- | --- | ------- | ---------- | --- | --- |
|        | Sub-index [8] |       |     |         | Length [8] |     |     |
| MSB    |               |       |     |         |            |     | LSB |
| Bit    | Notation      |       |     | Meaning |            |     |     |
0 to 7 Length This contains the length of the object to be mapped in units of bits.
| 8 to 15  | Sub-index This contains the sub-index of the object to be mapped. |     |     |     |     |     |     |
| -------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 16 to 31 | Index This contains the index of the object to be mapped.         |     |     |     |     |     |     |
86

## Page 87

Object Dictionary
z  1A03h: 4th TPDO mapping parameter
This object contains the mapping for the PDOs the driver is able to transmit. (TPDO4)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
4th TPDO mapping parameter
Number of mapped 
|     | 00h application objects in  |     | UINT8 | rw  | No  | 2   | – Yes |
| --- | --------------------------- | --- | ----- | --- | --- | --- | ----- |
TPDO
0000 0000h 
to 
|     | 01h 1st application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(6041 0010h)
0000 0000h 
to 
1A03h
|     | 02h 2nd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(606C 0020h)
0000 0000h 
to 
|     | 03h 3rd application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
0000 0000h 
to 
|     | 04h 4th application object |     | UINT32 | rw  | No  |     | – Yes |
| --- | -------------------------- | --- | ------ | --- | --- | --- | ----- |
FFFF FFFFh 
(0000 0000h)
Data Description of Sub-index 01h to 04h
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | ----- | ----- | ----- |
Index [16]
| Bit 15 | 14 13 12      | 11 10 | 9 8 | 7 6     | 5 4        | 3 2 | 1 0 |
| ------ | ------------- | ----- | --- | ------- | ---------- | --- | --- |
|        | Sub-index [8] |       |     |         | Length [8] |     |     |
| MSB    |               |       |     |         |            |     | LSB |
| Bit    | Notation      |       |     | Meaning |            |     |     |
0 to 7 Length This contains the length of the object to be mapped in units of bits.
| 8 to 15  | Sub-index This contains the sub-index of the object to be mapped. |     |     |     |     |     |     |
| -------- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| 16 to 31 | Index This contains the index of the object to be mapped.         |     |     |     |     |     |     |
87

## Page 88

Object Dictionary
8.2  Manufacturer Specific Objects
Refer to the following for details on the Manufacturer Specific Objects.
- OPERATING MANUAL BLV Series R Type Function Edition
z  402Ch: Direct data operation operation data number
This object is the operation data number to be used in direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Direct data operation 
| 402Ch | 00h | INT16 rww | Yes | 0 to 255 (0) | – No |
| ----- | --- | --------- | --- | ------------ | ---- |
operation data number
z  402Dh: Direct data operation operation type
This object is the operation type for direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Direct data operation 
| 402Dh | 00h | UINT8 rww | Yes | 0 to 255 (0) | – No |
| ----- | --- | --------- | --- | ------------ | ---- |
operation type
z  402Eh: Direct data operation position
This object is the target position for direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
–2,147,483,648 
|       | Direct data operation  |           |                |  to  | Pos.  |
| ----- | ---------------------- | --------- | -------------- | ---- | ----- |
| 402Eh | 00h                    | INT32 rww | Yes            |      | No    |
|       | position               |           | 2,147,483,647  |      | unit  |
(0)
z  402Fh: Direct data operation operating velocity
This object is the operating velocity for direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
–4,000,000 
|       | Direct data operation  |           |     |       | Vel.  |
| ----- | ---------------------- | --------- | --- | ----- | ----- |
| 402Fh | 00h                    | INT32 rww | Yes |  to   | No    |
|       | operating velocity     |           |     |       | unit  |
4,000,000 (0)
z  4030h: Direct data operation acceleration rate
This object is the acceleration rate (acceleration time) for direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1 
Acc. 
|       | Direct data operation  |           |                |  to  |          |
| ----- | ---------------------- | --------- | -------------- | ---- | -------- |
| 4030h | 00h                    | INT32 rww | Yes            |      | unit  No |
|       | acceleration rate      |           | 1,000,000,000  |      |          |
(MS)
(1,000)
z  4031h: Direct data operation deceleration rate
This object is the deceleration rate (deceleration time) for direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1 
Acc. 
|       | Direct data operation  |           |                |  to  |          |
| ----- | ---------------------- | --------- | -------------- | ---- | -------- |
| 4031h | 00h                    | INT32 rww | Yes            |      | unit  No |
|       | deceleration rate      |           | 1,000,000,000  |      |          |
(MS)
(1,000)
z  4032h: Direct data operation torque limiting value
This object is the torque limiting value for direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Direct data operation  |           |     | 0 to 10,000  |         |
| ----- | ---------------------- | --------- | --- | ------------ | ------- |
| 4032h | 00h                    | INT16 rww | Yes |              | 0.1% No |
|       | torque limiting value  |           |     | (10,000)     |         |
88

## Page 89

Object Dictionary
z  4033h: Direct data operation trigger
This object is the trigger for direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
–7 to 
Direct data operation 
| 4033h | 00h | INT32 | rww Yes 2,147,418,131  | –   | No  |
| ----- | --- | ----- | ---------------------- | --- | --- |
trigger
(0)
z  4034h: Direct data operation forwarding destination
This object is the stored area when the next direct data is transferred during direct data operation.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Direct data operation 
| 4034h | 00h | UINT8 | rww Yes | 0 to 1 (0) – | No  |
| ----- | --- | ----- | ------- | ------------ | --- |
forwarding destination
z  403Ah: Driver input command (2nd)
This object is the same input command as “Driver input command”.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0000 0000h 
|       | Driver input command  |        |         |  to         |     |
| ----- | --------------------- | ------ | ------- | ----------- | --- |
| 403Ah | 00h                   | UINT32 | rww Yes | –           | No  |
|       | (2nd)                 |        |         | FFFF FFFFh  |     |
(0000 0000h)
z  403Ch: Driver input command (automatic OFF)
This object is the same input command as “Driver input command”. If the input signal is turned ON with this 
command, it is automatically turned OFF after 250 μs.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0000 0000h 
|       | Driver input command  |        |         |  to         |     |
| ----- | --------------------- | ------ | ------- | ----------- | --- |
| 403Ch | 00h                   | UINT32 | rww Yes | –           | No  |
|       | (automatic OFF)       |        |         | FFFF FFFFh  |     |
(0000 0000h)
z  403Dh: NET selection data number
This object is the operation data number.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
NET selection data 
| 403Dh | 00h | UINT32 | rww Yes | 0 to 255 (0) – | No  |
| ----- | --- | ------ | ------- | -------------- | --- |
number
z  403Eh: Driver input command
This object is the input command to the driver.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0000 0000h 
 to  
| 403Eh | 00h Driver input command | UINT32 | rww Yes | –   | No  |
| ----- | ------------------------ | ------ | ------- | --- | --- |
FFFF FFFFh 
(0000 0000h)
z  403Fh: Driver output status
This object is the output status of the driver.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 403Fh | 00h Driver output status | UINT32 | ro Yes | – – | No  |
| ----- | ------------------------ | ------ | ------ | --- | --- |
89

## Page 90

Object Dictionary
z  404Bh: Target position (User-defined position unit)
This object is the present target position. The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Target position (User- |       |        | Pos.  |     |
| ----- | ---------------------- | ----- | ------ | ----- | --- |
| 404Bh | 00h                    | INT32 | ro Yes | –     | No  |
|       | defined position unit) |       |        | unit  |     |
z  404Ch: Demand position (User-defined position unit)
This object is the present demand position. The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Demand position 
Pos. 
| 404Ch | 00h (User-defined position  | INT32 | ro Yes | –   | No  |
| ----- | --------------------------- | ----- | ------ | --- | --- |
unit
unit)
z  404Dh: Actual position (User-defined position unit)
This object is the present actual position. The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Actual position (User- |       |        | Pos.  |     |
| ----- | ---------------------- | ----- | ------ | ----- | --- |
| 404Dh | 00h                    | INT32 | ro Yes | –     | No  |
|       | defined position unit) |       |        | unit  |     |
z  404Eh: Target velocity (User-defined velocity unit)
This object is the present target velocity. The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Target velocity (User- |       |        | Vel.  |     |
| ----- | ---------------------- | ----- | ------ | ----- | --- |
| 404Eh | 00h                    | INT32 | ro Yes | –     | No  |
|       | defined velocity unit) |       |        | unit  |     |
z  404Fh: Demand velocity (User-defined velocity unit)
This object is the present demand velocity. The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Demand velocity 
Vel. 
| 404Fh | 00h (User-defined velocity  | INT32 | ro Yes | –   | No  |
| ----- | --------------------------- | ----- | ------ | --- | --- |
unit
unit)
z  4050h: Actual velocity (User-defined velocity unit)
This object is the present demand velocity. The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Actual velocity (User- |       |        | Vel.  |     |
| ----- | ---------------------- | ----- | ------ | ----- | --- |
| 4050h | 00h                    | INT32 | ro Yes | –     | No  |
|       | defined velocity unit) |       |        | unit  |     |
z  4056h: Present communication error
This object is the communication error code received last time.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Present communication 
| 4056h | 00h | UINT8 | ro Yes | – – | No  |
| ----- | --- | ----- | ------ | --- | --- |
error
z  406Bh: Torque monitor
This object is the output torque presently generated as a percentage of the rated torque.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 406Bh | 00h Torque monitor | INT16 | ro Yes | – 0.1% | No  |
| ----- | ------------------ | ----- | ------ | ------ | --- |
90

## Page 91

Object Dictionary
z 406Ch: Load factor monitor
This object is the output torque presently generated as a percentage of the maximum torque in the continuous duty
region.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
406Ch 00h Load factor monitor INT32 ro Yes – 0.1% No
z 406Dh: Cumulative load monitor
This object is the integrated value of the load during operation. (Internal unit)
The load is accumulated regardless of the rotation direction of the motor.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Cumulative load
406Dh 00h UINT32 ro Yes – – No
monitor
z 4070h: Next data number
This object is the operation data number specified in “Next data number” of the operation data in operation. The value
is latched also after the operation is complete. When “Link” is “No Link” or “Next data number” is “Stop,” −1 is displayed.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
4070h 00h Next data number INT16 ro Yes – – No
z 4071h: Loop origin data number
This object is the operation data number that is the starting point of the loop in loop operation (extended loop
operation). When loop is not executed or stopped, −1 is displayed.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Loop origin data
4071h 00h INT16 ro Yes – – No
number
z 4072h: Loop count
This object is the current number of times of loop in loop operation (extended loop operation). When operation other
than loop is executed or loop is stopped, 0 is displayed.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
4072h 00h Loop count UINT32 ro Yes – – No
z 4073h: Position deviation
This object is the deviation between the demand position and actual position.
The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Pos.
4073h 00h Position deviation INT32 ro Yes – No
unit
z 4075h: Speed deviation
This object is the deviation between the demand position having input to the position controller and the actual
position. The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Vel.
4075h 00h Speed deviation INT32 ro Yes – No
unit
z 407Ah: Tripmeter 1
This object is the travel distance of the motor in revolutions. (1=0.1 krev)
This can be cleared on the customer side.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0.1
407Ah 00h Tripmeter 1 INT32 ro Yes – No
krev
91

## Page 92

Object Dictionary
z  407Bh: Present information
This object is the information status presently being generated.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 407Bh | 00h Information status 1 | UINT32 | ro Yes | – – | No  |
| ----- | ------------------------ | ------ | ------ | --- | --- |
z  407Ch: Driver temperature
This object is the present driver temperature. (1=0.1 °C)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 407Ch | 00h Driver temperature | INT16 | ro Yes | – 0.1 °C | No  |
| ----- | ---------------------- | ----- | ------ | -------- | --- |
z  407Dh: Motor temperature
This object is the present motor temperature. (1=0.1 °C)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 407Dh | 00h Motor temperature | INT16 | ro Yes | – 0.1 °C | No  |
| ----- | --------------------- | ----- | ------ | -------- | --- |
z  407Eh: Odometer
This object is the cumulative travel distance of the motor in revolutions. (1=0.1 krev)
This cannot be cleared on the customer side.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0.1 
| 407Eh | 00h Odometer | UINT32 | ro Yes | –   | No  |
| ----- | ------------ | ------ | ------ | --- | --- |
krev
z  407Fh: Tripmeter 0
This object is the travel distance of the motor in revolutions. (1=0.1 krev)
This can be cleared on the customer side.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0.1 
| 407Fh | 00h Tripmeter 0 | UINT32 | ro Yes | –   | No  |
| ----- | --------------- | ------ | ------ | --- | --- |
krev
z  409Bh: Main power supply current
This object is the present current value of the main power supply. (1=0.001 A)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Main power supply  |       |        | 0.001  |     |
| ----- | ------------------ | ----- | ------ | ------ | --- |
| 409Bh | 00h                | INT32 | ro Yes | –      | No  |
|       | current            |       |        | A      |     |
z  409Ch: Power consumption
This object is the present power consumption. (1=0.1 W)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 409Ch | 00h Power consumption | UINT32 | ro Yes | – 0.1 W | No  |
| ----- | --------------------- | ------ | ------ | ------- | --- |
z  409Dh: Energy consumption
This object is the present energy consumption. (1=0.001 Wh)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0.001 
| 409Dh | 00h Energy consumption | UINT32 | ro Yes | –   | No  |
| ----- | ---------------------- | ------ | ------ | --- | --- |
Wh
92

## Page 93

Object Dictionary
z 409Eh: User energy consumption
This object is the total energy consumption.
This can be cleared on the customer side.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
User energy
409Eh 00h UINT32 ro Yes – Wh No
consumption
z 409Fh: Total energy consumption
This object is Indicates the total energy consumption.
This cannot be cleared on the customer side.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Total energy
409Fh 00h UINT32 ro Yes – Wh No
consumption
z 40A1h: Total uptime
This object is the total time that has elapsed since the main power supply was turned on. (min)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40A1h 00h Total uptime UINT32 ro Yes – min No
z 40A2h: Number of boots
This object is the total number of times that the driver was started.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40A2h 00h Number of boots UINT32 ro Yes – – No
z 40A3h: Inverter voltage
This object is the inverter voltage of the driver. (1=0.1 V)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40A3h 00h Inverter voltage INT16 ro Yes – 0.1 V No
z 40A4h: Main power supply voltage
This object is the main power supply voltage. (1=0.1 V)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Main power supply
40A4h 00h INT16 ro Yes – 0.1 V No
voltage
z 40A9h: Continuous uptime
This object is the time at which the main power supply is supplied continuously. (ms)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40A9h 00h Continuous uptime UINT32 ro Yes – ms No
z 40AAh: RS-485 communication reception byte counter
This object is the number of bytes received.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
RS-485 communication
40AAh 00h UINT32 ro Yes – – No
reception byte counter
93

## Page 94

Object Dictionary
z 40ABh: RS-485 communication transmission byte counter
This object is the number of bytes transmitted.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
RS-485 communication
40ABh 00h transmission byte UINT32 ro Yes – – No
counter
z 40C0h: Alarm reset
This object resets the alarm being generated presently. Some alarms cannot be reset.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40C0h 00h Alarm reset UINT8 rww Yes 0 to 2 (0) – No
z 40C2h: Clear alarm history
This object clears the alarm history.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40C2h 00h Clear alarm history UINT8 rww Yes 0 to 2 (0) – No
z 40C5h: P-PRESET execution
This object is presets the demand position.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40C5h 00h P-PRESET execution UINT8 rww Yes 0 to 2 (0) – No
z 40C6h: Configuration
This object executes recalculation and setup of the parameter.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40C6h 00h Configuration UINT8 rww Yes 0 to 2 (0) – No
z 40CDh: Clear latch information
This object clears latch information.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40CDh 00h Clear latch information UINT8 rww Yes 0 to 2 (0) – No
z 40CEh: Clear sequence history
This object clears sequence history.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40CEh 00h Clear sequence history UINT8 rww Yes 0 to 2 (0) – No
z 40D0h: Clear ETO
This object releases the ETO status.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40D0h 00h Clear ETO UINT8 rww Yes 0 to 2 (0) – No
z 40D1h: ZSG-PRESET
This object sets the position of the ZSG-N output again.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
40D1h 00h ZSG-PRESET UINT8 rww Yes 0 to 2 (0) – No
94

## Page 95

Object Dictionary
z  40D2h: Clear ZSG-PRESET
This object clears the position data of the ZSG-N output that was set again with the “ZSG-PRESET command.”
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 40D2h | 00h Clear ZSG-PRESET | UINT8 rww | Yes 0 to 2 (0) | – No |
| ----- | -------------------- | --------- | -------------- | ---- |
z  40D3h: Clear information
This object clears the information.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 40D3h | 00h Clear information | UINT8 rww | Yes 0 to 2 (0) | – No |
| ----- | --------------------- | --------- | -------------- | ---- |
z  40D6h: Clear user energy consumption
This object clears the user energy consumption.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Clear user energy 
| 40D6h | 00h | UINT8 rww | Yes 0 to 2 (0) | – No |
| ----- | --- | --------- | -------------- | ---- |
consumption
z  40D7h: Clear tripmeter 0
This object clear tripmeter 0
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 40D7h | 00h Clear tripmeter 0 | UINT8 rww | Yes 0 to 2 (0) | – No |
| ----- | --------------------- | --------- | -------------- | ---- |
z  40D8h: Clear tripmeter 1
This object clear tripmeter 1
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 40D8h | 00h Clear tripmeter 1 | UINT8 rww | Yes 0 to 2 (0) | – No |
| ----- | --------------------- | --------- | -------------- | ---- |
z  4148h: Permission of absolute positioning without setting absolute coordinates
This object permits absolute positioning operation in a state where the position coordinate has not been set.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Permission of absolute 
posi-tioning without 
| 4148h | 00h | UINT8 rww | Yes 0 to 1 (0) | – No |
| ----- | --- | --------- | -------------- | ---- |
setting absolute 
coordinates
z  415Fh: JOG/HOME Torque limit value
This object is the torque limiting value.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | JOG/HOME Torque limit  |            | 0 to 10,000  |          |
| ----- | ---------------------- | ---------- | ------------ | -------- |
| 415Fh | 00h                    | UINT16 rww | Yes          | 0.1% Yes |
value (10,000)
z  4160h: (HOME) Homing mode
This object is the homing method.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 4160h | 00h (HOME) Homing mode | UINT8 rww | Yes 0 to 3 (1) | – Yes |
| ----- | ---------------------- | --------- | -------------- | ----- |
z  4163h: (HOME) Starting velocity
This object is the starting velocity. The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | (HOME) Starting  |            | 1 to 4,000,000  | Vel.  |
| ----- | ---------------- | ---------- | --------------- | ----- |
| 4163h | 00h              | UINT32 rww | Yes             | Yes   |
|       | velocity         |            | (30)            | unit  |
95

## Page 96

Object Dictionary
z 4169h: (HOME) Backward steps in 2 sensor homeseeking
This object is the amount of backward steps after homing operation in 2-sensor mode.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
(HOME) Backward steps
0 to 8,388,607 Pos.
4169h 00h in 2 sensor UINT32 rww Yes Yes
(18,000) unit
homeseeking
z 4186h: Stopping timeout at alarm generation
This object is the time-out period from when the alarm of “Non-excitation after deceleration” is generated until the
excitation is turned off.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Stopping timeout at 0 to 10,000
4186h 00h UINT16 rww Yes ms Yes
alarm generation (3,000)
z 41CAh: WRAP setting
This object is the WRAP setting.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
41CAh 00h WRAP setting UINT8 rww Yes 1 to 2 (1) – Yes
z 4735h: Custom stopping rate
This object is the deceleration rate when select the “Deceleration rate stop (according to the Custom stopping rate
parameter)” in parameter of “STOP input action” or “QSTOP input action”.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1 to
Acc.
4735h 00h Custom stopping rate UINT32 rww Yes 1,000,000,000 Yes
unit
(1,000)
z 4736h: Custom stopping time
This object is the deceleration time when select the “Deceleration time stop (according to the Custom stopping time
parameter)” in parameter of “STOP input action” or “QSTOP input action”.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1 to
4736h 00h Custom stopping time UINT32 rww Yes 1,000,000,000 ms Yes
(1,000)
96

## Page 97

Object Dictionary
8.3  Device Profile Objects
z  603Fh: Error code
This object provides the error code of the last error that occurred. (The latest alarm history of the driver + FF00h.)
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 603Fh | 00h Error code |     | UINT16 | ro  | Yes | –   | – No |
| ----- | -------------- | --- | ------ | --- | --- | --- | ---- |
z  6040h: Controlword
This object controls the driver and operation mode.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0000h to FFFFh 
| 6040h | 00h Controlword |     | UINT16 | rww | Yes |     | – No |
| ----- | --------------- | --- | ------ | --- | --- | --- | ---- |
(0004h)
Data Description
| Bit 15 | 14 13             | 12 11 10 | 9 8      | 7   | 6 5 4       | 3 2   | 1 0   |
| ------ | ----------------- | -------- | -------- | --- | ----------- | ----- | ----- |
|        | MS [5]            | RSV      | OMS HALT | FR  | OMS [3]     | EO QS | EV SO |
| MSB    |                   |          |          |     |             |       | LSB   |
| Bit    | Notation          | Meaning  |          |     | Description |       |       |
| 0      | SO Switch on      |          |          |     |             |       |       |
| 1      | EV Enable voltage |          |          |     |             |       |       |
Status Machine control commands
| 2   | QS Quick stop       |     |     |     |     |     |     |
| --- | ------------------- | --- | --- | --- | --- | --- | --- |
| 3   | EO Enable operation |     |     |     |     |     |     |
4 to 6 OMS Operation mode specific For details, refer to each operation mode.
| 7   | FR Fault reset |     | 0 -> 1: Alarm reset |     |     |     |     |
| --- | -------------- | --- | ------------------- | --- | --- | --- | --- |
| 8   | HALT Halt      |     |                     |     |     |     |     |
For details, refer to each operation mode.
| 9   | OMS Operation mode specific |     |           |     |     |     |     |
| --- | --------------------------- | --- | --------- | --- | --- | --- | --- |
| 10  | RSV Reserved                |     | Reserved  |     |     |     |     |
Manufacturer-specific bit 
| 11 to 15 | MS Manufacturer specific |     |     |     |     |     |     |
| -------- | ------------------------ | --- | --- | --- | --- | --- | --- |
For details, refer to each operation mode.
Operation mode
- Profile Velocity Mode (pv)
- Profile Position Mode (pp)
- Profile Torque Mode (tq)
- Homing Mode (hm)
97

## Page 98

Object Dictionary
z  6041h: Statusword
This object contains the bits that provide the current state of the driver and the operating state of the operation 
mode.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 6041h |     | 00h Statusword |     | UINT16 | ro  | Yes | –   | –   | No  |
| ----- | --- | -------------- | --- | ------ | --- | --- | --- | --- | --- |
Data Description
| Bit | 15 14    | 13                 | 12 11 10 | 9 8                          | 7 6   | 5 4         | 3 2      | 1   | 0    |
| --- | -------- | ------------------ | -------- | ---------------------------- | ----- | ----------- | -------- | --- | ---- |
|     | MS [2]   | OMS [2]            | ILA TR   | RM RDY                       | W SOD | QS VE       | FAULT OE | SO  | RTSO |
|     | MSB      |                    |          |                              |       |             |          |     | LSB  |
| Bit | Notation |                    | Meaning  |                              |       | Description |          |     |      |
|     | 0 RTSO   | Ready to switch on |          |                              |       |             |          |     |      |
|     | 1 SO     | Switch on          |          |                              |       |             |          |     |      |
|     | 2 OE     | Operation enabled  |          |                              |       |             |          |     |      |
|     | 3 FAULT  | Fault              |          | Current state of the driver. |       |             |          |     |      |
|     | 4 VE     | Voltage enabled    |          |                              |       |             |          |     |      |
|     | 5 QS     | Quick stop         |          |                              |       |             |          |     |      |
|     | 6 SOD    | Switch on disabled |          |                              |       |             |          |     |      |
0: No alarm occurred 
|     | 7 W | Warning |     |     |     |     |     |     |     |
| --- | --- | ------- | --- | --- | --- | --- | --- | --- | --- |
1: Alarm occurred
Manufacturer-specific bit 
|     | 8 MS | Manufacturer specific |     |     |     |     |     |     |     |
| --- | ---- | --------------------- | --- | --- | --- | --- | --- | --- | --- |
For details, refer to each operation mode.
0: Controlword is not processed. * 
|     | 9 RM | Remote |     |     |     |     |     |     |     |
| --- | ---- | ------ | --- | --- | --- | --- | --- | --- | --- |
1: Controlword is processed.
10 TR Target reached For details, refer to each operation mode.
The internal limit is activated in the following cases: 
 - The software limit was activated. 
 - The FW-LS or RV-LS signal was activated. 
|     | 11 ILA | Internal limit active |     |     |     |     |     |     |     |
| --- | ------ | --------------------- | --- | --- | --- | --- | --- | --- | --- |
 - The FW-BLK or RV-BLK signal was activated. 
 - The STOP or QSTOP signal was activated. 
 - The CLR signal was activated.
12 to 13 OMS Operation mode specific For details, refer to each operation mode.
Manufacturer-specific bit 
| 14 to 15 | MS  | Manufacturer specific |     |     |     |     |     |     |     |
| -------- | --- | --------------------- | --- | --- | --- | --- | --- | --- | --- |
For details, refer to each operation mode.
* The Remote (bit 9) is “0” when any of the following conditions. 
- The S-ON signal is active. 
- Remote operation, data writing, or I/O test is executed with the support soft.
Operation mode
- Profile Velocity Mode (pv)
- Profile Position Mode (pp)
- Profile Torque Mode (tq)
- Homing Mode (hm)
98

## Page 99

Object Dictionary
z 605Ah: Quick stop option code
This object determines what operation will be performed if a Quick Stop is executed.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
605Ah 00h Quick stop option code INT16 rw No –3 to 6 (2) – No
Value Description
–3 Decelerates with the Custom stopping time (4736h) and stay in Quick stop active state.
–2 Decelerates with the Custom stopping rate (4735h) and stay in Quick stop active state.
–1 Immediate stop and stay in Quick stop active state.
0 Immediate stop and transit into Switch on disabled state.
Decelerates with slow down ramp (deceleration ramp depending on operating mode) and transit into
1
Switch on disabled state.
2 Decelerates with quick stop ramp (6085h) and transit into Switch on disabled state.
Decelerates with slow down ramp (deceleration ramp depending on operating mode) and stay in Quick
5
stop active state.
6 Decelerates with quick stop ramp (6085h) and stay in Quick stop active state.
z 605Bh: Shutdown option code
This object defines the operation that is performed if there is a transition from Operation enable state to Ready to
switch on state.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
605Bh 00h Shutdown option code INT16 rw No 0 to 1 (0) – Yes
Value Description
0 Immediate stop and transit into Ready to switch on state.
Decelerates with slow down ramp (deceleration ramp depending on operating mode) and transit into
1
Ready to switch on state.
z 605Ch: Disable operation option code
This object defines the operation that is performed if there is a transition from Operation enable state to Switched on
state.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Disable operation
605Ch 00h INT16 rw No 0 to 1 (1) – Yes
option code
Value Description
0 Immediate stop and transit into Switched on state.
Decelerates with slow down ramp (deceleration ramp depending on operating mode) and transit into
1
Switched on state.
z 605Dh: Halt option code
This object defines the operation that is performed if bit 8 (Halt) in controlword is active.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
605Dh 00h Halt option code INT16 rw No 0 to 1 (1) – Yes
Value Description
0 Reserved
Decelerates with slow down ramp (deceleration ramp depending on operating mode except torque
1
limit value).
99

## Page 100

Object Dictionary
z 605Eh: Fault reaction option code
This object defines the operation that is performed when an alarm is detected in the driver.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Fault reaction option
605Eh 00h INT16 rw No 0 to 2 (2) – Yes
code
Value Description
0 Immediate stop and motor will be non-excitation.
1 Decelerates with slow down ramp (deceleration ramp depending on operating mode).
2 Decelerates with quick stop ramp (6085h).
z 6060h: Modes of operation
This object is used to select the operation mode. The driver provides the actual operation mode in the modes of
operation display object (6061h).
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
6060h 00h Modes of operation INT8 rww Yes 0 to 6 (3) – Yes
Value Description
0 There is no mode change or no mode assigned.
1 Profile Position Mode
3 Profile Velocity Mode
4 Profile Torque Mode
6 Homing Mode
z 6061h: Modes of operation display
This object provides the actual operation mode.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Modes of operation
6061h 00h INT8 ro Yes – – No
display
z 6062h: Position demand value
This object provides the position demand value in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Pos.
6062h 00h Position demand value INT32 ro Yes – No
unit
z 6064h: Position actual value
This object provides the position actual value in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Pos.
6064h 00h Position actual value INT32 ro Yes – No
unit
z 6065h: Following error window
This object defines the detection range for the following error.
The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0 to 10,000,000 Pos.
6065h 00h Following error window UINT32 rww Yes Yes
(108,000) unit
100

## Page 101

Object Dictionary
z 6067h: Position window
This object defines the configured symmetrical range of accepted positions relative to the target position.
If the actual position value is within the position window, this target position is regarded as having been reached (bit
10 (target reached) in statusword is set to 1). The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Pos.
6067h 00h Position window UINT32 rww Yes 0 to 65,535 (18) Yes
unit
z 606Bh: Velocity demand value
This object provides the velocity demand value in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Vel.
606Bh 00h Velocity demand value INT32 ro Yes – No
unit
z 606Ch: Velocity actual value
This object provides the velocity actual value in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Vel.
606Ch 00h Velocity actual value INT32 ro Yes – No
unit
z 606Dh: Velocity window
This object defines the configured symmetrical range of accepted velocities relative to the target velocity.
If the actual velocity value is within the velocity window, this target velocity is regarded as having been reached (bit
10 (target reached) in statusword is set to 1). The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Vel.
606Dh 00h Velocity window UINT16 rww Yes 1 to 65,535 (15) Yes
unit
z 606Fh: Velocity threshold
This object defines the configured symmetrical range of accepted velocities relative to the zero.
The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Vel.
606Fh 00h Velocity threshold UINT16 rww Yes 1 to 65,535 (15) Yes
unit
z 6071h: Target torque
This object contains the target torque for the Profile Torque Mode.
The value is given per thousand of rated torque.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
-1,000 to 1,000
6071h 00h Target torque INT16 rww Yes 0.1% No
(0)
z 6072h: Max torque
This object defines the configured maximum permissible torque in the motor.
The value is given per thousand of rated torque.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0 to 10,000
6072h 00h Max torque UINT16 rww Yes 0.1% Yes
(10,000)
101

## Page 102

Object Dictionary
z  6074h: Torque demand
This object provides the torque demand value. The value is given per thousand of rated torque.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 6074h | 00h Torque demand | INT16 | ro Yes | – 0.1% | No  |
| ----- | ----------------- | ----- | ------ | ------ | --- |
z  6077h: Torque actual value
This object provides the actual value of the torque. It is correspond to the instantaneous torque in the motor. 
The value is given per thousand of rated torque.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 6077h | 00h Torque actual value | INT16 | ro Yes | – 0.1% | No  |
| ----- | ----------------------- | ----- | ------ | ------ | --- |
z  607Ah: Target position
This object contains the target position for the Profile Position Mode or Profile Velocity Mode.
In Profile Position Mode, the value of this object is interpreted as either an absolute or relative value depending on 
the Abs/Rel Flag in controlword. 
In Profile Velocity Mode, the value is always interpreted as an absolute value.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
–2,147,483,648 
|       |                     |       |                | to  Pos.  |     |
| ----- | ------------------- | ----- | -------------- | --------- | --- |
| 607Ah | 00h Target position | INT32 | rww Yes        |           | No  |
|       |                     |       | 2,147,483,647  | unit      |     |
(0)
z  607Bh: Position range limit
This object is used to define the start and end of the range of movement for modulo axis.
The start of the range is defined by Sub-index 01h (Min position range limit) and the end by Sub-index 02h (Max 
position range limit). 
To do this, Object 41CAh (WRAP setting) must have the value 2 applied.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Position range limit
Highest Sub-index 
|     | 00h | UINT8 | c No | 2  – | No  |
| --- | --- | ----- | ---- | ---- | --- |
Supported
–2,147,483,648 
Pos. 
|       | 01h Min position range limit | INT32 | rww Yes | to   | Yes |
| ----- | ---------------------------- | ----- | ------- | ---- | --- |
| 607Bh |                              |       |         | unit |     |
0 (0)
0 
|     |                              |       |                |  to  Pos.  |     |
| --- | ---------------------------- | ----- | -------------- | ---------- | --- |
|     | 02h Max position range limit | INT32 | rww Yes        |            | Yes |
|     |                              |       | 2,147,483,648  | unit       |     |
(0)
z  607Ch: Home offset
This object is the configured difference between the zero position for the application and the machine home position 
(found during homing). During homing, the machine home position is found and once the homing is completed, the 
zero position is offset from the home position by adding the home offset to the home position.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
–2,147,483,648 
|       |                 |       |                | to  Pos.  |     |
| ----- | --------------- | ----- | -------------- | --------- | --- |
| 607Ch | 00h Home offset | INT32 | rww Yes        |           | Yes |
|       |                 |       | 2,147,483,647  | unit      |     |
(0)
Home Zero
position position
  Home offset (607Ch)
102

## Page 103

Object Dictionary
z  607Dh: Software position limit 
This object defines the absolute positions of the limits to the target position (position demand value). 
Every target position is checked against these limits. The limit positions are specified in user-defined position units, 
the same as for target positions, and are always relative to the machine home position.
The limit values are corrected internally for the home offset as given below. The target positions are compared with 
the corrected values.
- Corrected minimum position limit = Min position limit – Home offset (607Ch)
- Corrected maximum position limit = Max position limit – Home offset (607Ch)
The software position limits are enabled at the following times:
- When homing is completed
The software limits are disabled if they are set as follows:
- Min position limit ≥ Max position limit
- Min position limit and Max position limit are set to “0”
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Software position limit 
Highest Sub-index 
|     | 00h | UINT8 | c No | 2  – | No  |
| --- | --- | ----- | ---- | ---- | --- |
Supported
–2,147,483,648 
|       |                        |       |                | to  Pos.  |     |
| ----- | ---------------------- | ----- | -------------- | --------- | --- |
| 607Dh | 01h Min position limit | INT32 | rww Yes        |           | Yes |
|       |                        |       | 2,147,483,647  | unit      |     |
(0)
–2,147,483,648 
|     |                        |       |                | to  Pos.  |     |
| --- | ---------------------- | ----- | -------------- | --------- | --- |
|     | 02h Max position limit | INT32 | rww Yes        |           | Yes |
|     |                        |       | 2,147,483,647  | unit      |     |
(0)
Refer to the following for details on the software position limit.
- OPERATING MANUAL BLV Series R Type Function Edition 
z  6081h: Profile velocity
This object is the final movement velocity at the end of acceleration for the Profile Position Mode or Profile Velocity 
Mode.
The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       |                      |        | 1 to 4,000,000  | Vel.     |     |
| ----- | -------------------- | ------ | --------------- | -------- | --- |
| 6081h | 00h Profile velocity | UINT32 | rww Yes         |          | Yes |
|       |                      |        |                 | (1) unit |     |
z  6082h: End velocity
This object is the velocity at start and end of the ramp for the Profile Position Mode.
The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       |                  |        | 0 to 4,000,000  | Vel.     |     |
| ----- | ---------------- | ------ | --------------- | -------- | --- |
| 6082h | 00h End velocity | UINT32 | rww Yes         |          | Yes |
|       |                  |        |                 | (0) unit |     |
z  6083h: Profile acceleration
This object is the acceleration rate for the Profile Position Mode or Profile Velocity Mode.
The value is given in user-defined acceleration units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1 
|       |                          |        |                |  to  Acc.  |     |
| ----- | ------------------------ | ------ | -------------- | ---------- | --- |
| 6083h | 00h Profile acceleration | UINT32 | rww Yes        |            | Yes |
|       |                          |        | 1,000,000,000  | unit       |     |
(1,000)
103

## Page 104

Object Dictionary
z  6084h: Profile deceleration
This object is the deceleration rate for the Profile Position Mode or Profile Velocity Mode.
The value is given in user-defined acceleration units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1 
|       |                          |        |                |  to  Acc.  |     |
| ----- | ------------------------ | ------ | -------------- | ---------- | --- |
| 6084h | 00h Profile deceleration | UINT32 | rww Yes        |            | Yes |
|       |                          |        | 1,000,000,000  | unit       |     |
(1,000)
z  6085h: Quick stop deceleration
This object is the configured deceleration used to stop the motor when the object “Quick stop code (605Ah)” is set to 
“2” or “6”. 
The quick stop deceleration is also used if the object “Fault reaction code (605Eh)” is set to “2”. 
The value is given in user-defined acceleration units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1 
|       |                             |        |                |  to  Acc.  |     |
| ----- | --------------------------- | ------ | -------------- | ---------- | --- |
| 6085h | 00h Quick stop deceleration | UINT32 | rww Yes        |            | Yes |
|       |                             |        | 1,000,000,000  | unit       |     |
(1,000)
z  6087h: Torque slope
This object is the configured rate of change of torque. 
The value is given in units of per thousand of rated torque per second.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0 to 1,000,000 
| 6087h | 00h Torque slope | UINT32 | rww Yes | 0.1%/s | Yes |
| ----- | ---------------- | ------ | ------- | ------ | --- |
(0)
z  608Fh: Position encoder resolution
This object is the configured encoder increments and the number of motor revolutions. 
The control resolution is calculated by the following formula: 
Encoder increments
| Control resolution | =   |     |     |     |     |
| ------------------ | --- | --- | --- | --- | --- |
Motor revolutions
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Position encoder resolution
Highest Sub-index 
|     | 00h | UINT8 | c No | 2  – | No  |
| --- | --- | ----- | ---- | ---- | --- |
Supported
608Fh
1 to 65,535 
|     | 01h Encoder increments | UINT32 | rww Yes | –   | Yes |
| --- | ---------------------- | ------ | ------- | --- | --- |
(36,000)
|     | 02h Motor revolutions | UINT32 | rww Yes 1 to 65,535 (1) | –   | Yes |
| --- | --------------------- | ------ | ----------------------- | --- | --- |
Refer to the following for details on the control resolution.
- OPERATING MANUAL BLV Series R Type Function Edition
104

## Page 105

Object Dictionary
z  6091h: Gear ratio
This object is the configured number of motor shaft revolutions and the number of driving shaft revolutions. 
The gear ratio is calculated by the following formula:
Motor shaft revolutions
| Gear ratio | =   |     |     |     |     |
| ---------- | --- | --- | --- | --- | --- |
driving shaft revolutions
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Gear ratio
Highest sub-index 
|     | 00h | UINT8 | c No | 2   | – No |
| --- | --- | ----- | ---- | --- | ---- |
supported
6091h
|     | 01h Motor revolutions | UINT32 | rw No | 1 to 1,000 (1) | – Yes |
| --- | --------------------- | ------ | ----- | -------------- | ----- |
|     | 02h Shaft revolutions | UINT32 | rw No | 1 to 1,000 (1) | – Yes |
Refer to the following for details on the gear ratio.
- OPERATING MANUAL BLV Series R Type Function Edition
z  6098h: Homing method
This object is the homing method. 
Refer to the following section for details on the operations that are performed.
- “7.5.5 Homing method”
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 6098h | 00h Homing method | INT8 | rww Yes | –1 to 37 (37) | – Yes |
| ----- | ----------------- | ---- | ------- | ------------- | ----- |
Data Description
| Value |                                                                              | Description |     |     |     |
| ----- | ---------------------------------------------------------------------------- | ----------- | --- | --- | --- |
| 1     | Homing on negative limit switch and index pulse                              |             |     |     |     |
| 2     | Homing on positive limit switch and index pulse                              |             |     |     |     |
| 8     | Homing on home switch and index pulse and starting in the positive direction |             |     |     |     |
12 Homing on home switch and index pulse and starting in the negative direction
| 17       | Homing on negative limit switch                              |     |     |     |     |
| -------- | ------------------------------------------------------------ | --- | --- | --- | --- |
| 18       | Homing on positive limit switch                              |     |     |     |     |
| 24       | Homing on home switch and starting in the positive direction |     |     |     |     |
| 28       | Homing on home switch and starting in the negative direction |     |     |     |     |
| 35, 37 * | Homing on current position                                   |     |     |     |     |
Homing method of Orientalmotor specifications. 
| –1  | Refer to the following for details on the operations.  |     |     |     |     |
| --- | ------------------------------------------------------ | --- | --- | --- | --- |
- OPERATING MANUAL BLV Series R Type Function Edition
* 35 and 37 perform the same action
z  6099h: Homing speeds
This object defines the speeds that are used during homing.
The speeds are given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Homing speeds
Highest sub-index 
|     | 00h | UINT8 | c No | 2   | – No |
| --- | --- | ----- | ---- | --- | ---- |
supported
6099h
|     | Speed during search for  |        |         | 1 to 4,000,000  |       |
| --- | ------------------------ | ------ | ------- | --------------- | ----- |
|     | 00h                      | UINT32 | rww Yes |                 | – Yes |
|     | switch                   |        |         | (60)            |       |
|     | Speed during search for  |        |         | 1 to 4,000,000  |       |
|     | 01h                      | UINT32 | rww Yes |                 | – Yes |
|     | zero                     |        |         | (30)            |       |
105

## Page 106

Object Dictionary
z  609Ah: Homing acceleration
This object defines the acceleration that is used during homing. 
The rate is given in user-defined acceleration units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
1 
|       |                         |     |     |        |     |     |  to            | Acc.  |     |
| ----- | ----------------------- | --- | --- | ------ | --- | --- | -------------- | ----- | --- |
| 609Ah | 00h Homing acceleration |     |     | UINT32 | rw  | Yes |                |       | Yes |
|       |                         |     |     |        |     |     | 1,000,000,000  | unit  |     |
(1,000)
z  60A8h: SI unit position
This object is the user-defined position units. This object does not reflect the writing value.
It can be used to monitor the current user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 60A8h | 00h SI unit position |     |     | UINT32 | rw  | Yes | –   | –   | No  |
| ----- | -------------------- | --- | --- | ------ | --- | --- | --- | --- | --- |
Data Description
| Bit 31 | 30 29 28                 | 27  | 26 25 | 24  | 23 22 | 21 20                  | 19 18 | 17  | 16  |
| ------ | ------------------------ | --- | ----- | --- | ----- | ---------------------- | ----- | --- | --- |
|        | Prefix [8]               |     |       |     |       | SI units numerator [8] |       |     |     |
| Bit 15 | 14 13 12                 | 11  | 10    | 9 8 | 7 6   | 5 4                    | 3 2   | 1   | 0   |
|        | SI units denominator [8] |     |       |     |       |                        | 00h   |     |     |
| MSB    |                          |     |       |     |       |                        |       |     | LSB |
Prefix for SI units
| Factor | Value |     | Factor |     | Value |     |     |     |     |
| ------ | ----- | --- | ------ | --- | ----- | --- | --- | --- | --- |
| 106    |       |     | 10-6   |     |       |     |     |     |     |
|        | 06h   |     |        |     | FAh   |     |     |     |     |
| 105    |       |     | 10-5   |     |       |     |     |     |     |
|        | 05h   |     |        |     | FBh   |     |     |     |     |
| 104    |       |     | 10-4   |     |       |     |     |     |     |
|        | 04h   |     |        |     | FCh   |     |     |     |     |
| 103    | 03h   |     | 10-3   |     | FDh   |     |     |     |     |
| 102    | 02h   |     | 10-2   |     | FEh   |     |     |     |     |
| 101    | 01h   |     | 10-1   |     | FFh   |     |     |     |     |
| 100    | 00h   |     |        |     |       |     |     |     |     |
SI units
| Value | Unit symbol |                                           |     | Description |     |     |     |     |     |
| ----- | ----------- | ----------------------------------------- | --- | ----------- | --- | --- | --- | --- | --- |
| 00h   | –           | Dimensionless                             |     |             |     |     |     |     |     |
| 01h   | m           | Meter                                     |     |             |     |     |     |     |     |
| 41h   | °           | Degree                                    |     |             |     |     |     |     |     |
| B4h   | rev         | Mechanical revolution                     |     |             |     |     |     |     |     |
| B5h   | step        | Encoder Increments                        |     |             |     |     |     |     |     |
| C0h   | rev         | Revolution (motor shaft)                  |     |             |     |     |     |     |     |
| D0h   | rev         | Revolution (driving shaft of the gearbox) |     |             |     |     |     |     |     |
| D1h   | °           | Degree (driving shaft of the gearbox)     |     |             |     |     |     |     |     |
106

## Page 107

Object Dictionary
z  60A9h: SI unit velocity
This object is the user-defined velocity units. This object does not reflect the writing value.
It can be used to monitor the current user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 60A9h | 00h SI unit velocity |     |     | UINT32 | rw  | Yes | –   | –   | No  |
| ----- | -------------------- | --- | --- | ------ | --- | --- | --- | --- | --- |
Data Description
| Bit 31 | 30 29 28                 | 27  | 26 25 | 24  | 23 22 | 21 20                  | 19 18 | 17  | 16  |
| ------ | ------------------------ | --- | ----- | --- | ----- | ---------------------- | ----- | --- | --- |
|        | Prefix [8]               |     |       |     |       | SI units numerator [8] |       |     |     |
| Bit 15 | 14 13 12                 | 11  | 10    | 9 8 | 7 6   | 5 4                    | 3 2   | 1   | 0   |
|        | SI units denominator [8] |     |       |     |       |                        | 00h   |     |     |
| MSB    |                          |     |       |     |       |                        |       |     | LSB |
Prefix for SI units
| Factor | Value |     | Factor | Value |     |     |     |     |     |
| ------ | ----- | --- | ------ | ----- | --- | --- | --- | --- | --- |
| 106    | 06h   |     | 10-6   |       | FAh |     |     |     |     |
| 105    | 05h   |     | 10-5   |       | FBh |     |     |     |     |
| 104    | 04h   |     | 10-4   |       | FCh |     |     |     |     |
| 103    | 03h   |     | 10-3   |       | FDh |     |     |     |     |
| 102    | 02h   |     | 10-2   |       | FEh |     |     |     |     |
| 101    | 01h   |     | 10-1   |       | FFh |     |     |     |     |
100
00h
SI units
| Value | Unit symbol |                                             |     | Description |     |     |     |     |     |
| ----- | ----------- | ------------------------------------------- | --- | ----------- | --- | --- | --- | --- | --- |
| 00h   | –           | Dimensionless or User-defined position unit |     |             |     |     |     |     |     |
| 01h   | m           | Meter                                       |     |             |     |     |     |     |     |
| 03h   | s           | Second                                      |     |             |     |     |     |     |     |
| 41h   | °           | Degree                                      |     |             |     |     |     |     |     |
| 47h   | min         | Minute                                      |     |             |     |     |     |     |     |
| B4h   | rev         | Mechanical revolution                       |     |             |     |     |     |     |     |
| B5h   | inc         | Encoder Increments                          |     |             |     |     |     |     |     |
| C0h   | rev         | Revolution (motor shaft)                    |     |             |     |     |     |     |     |
| D0h   | rev         | Revolution (driving shaft of the gearbox)   |     |             |     |     |     |     |     |
| D1h   | °           | Degree (driving shaft of the gearbox)       |     |             |     |     |     |     |     |
107

## Page 108

Object Dictionary
z 60B8h: Touch probe function
This object sets the touch probes.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0000h to FFFFh
60B8h 00h Touch probe function UINT16 rww Yes – Yes
(3131h)
Data Description
Value Description
0: Disables touch probe 1.
0
1: Enables touch probe 1.
0: Trigger first event of touch probe 1.
1
1: Continuous of touch probe 1.
0: Triggers on probe 1 input (USR-LAT-IN0 signal).
2
1: Triggers on ZSG-N signal.
3 Reserved
0: Stops sampling at positive edge of touch probe 1.
4
1: Starts sampling at positive edge of touch probe 1.
0: Stops sampling at negative edge of touch probe 1.
5
1: Starts sampling at negative edge of touch probe 1.
6 to 7 Reserved
0: Disables touch probe 2.
8
1: Enables touch probe 2.
0: Trigger first event of touch probe 2.
9
1: Continuous of touch probe 2.
0: Triggers on probe 2 input (USR-LAT-IN1 signal).
10
1: Triggers on ZSG-N signal.
11 Reserved
0: Stops sampling at positive edge of touch probe 2.
12
1: Starts sampling at positive edge of touch probe 2.
0: Stops sampling at negative edge of touch probe 2.
13
1: Starts sampling at negative edge of touch probe 2.
14 to 15 Reserved
108

## Page 109

Object Dictionary
z  60B9h: Touch probe status
This object is the status of the touch probes.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 60B9h | 00h Touch probe status | UINT16 | ro Yes | – – | No  |
| ----- | ---------------------- | ------ | ------ | --- | --- |
Data Description
Value Description
0: Touch probe 1 is disabled. 
0
1: Touch probe 1 is enabled.
0: Touch probe 1 no positive edge value stored. 
1
1: Touch probe 1 positive edge position stored.
0: Touch probe 1 no negative edge value stored. 
2
1: Touch probe 1 negative edge position stored.
| 3 to 7 | Reserved |     |     |     |     |
| ------ | -------- | --- | --- | --- | --- |
0: Touch probe 2 is disabled. 
8
1: Touch probe 2 is enabled.
0: Touch probe 2 no positive edge value stored. 
9
1: Touch probe 2 positive edge position stored.
0: Touch probe 2 no negative edge value stored. 
10
1: Touch probe 2 negative edge position stored.
| 11 to 15 | Reserved |     |     |     |     |
| -------- | -------- | --- | --- | --- | --- |
z  60BAh: Touch probe 1 positive edge
This object provides the position value of the touch probe 1 at positive edge.
The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Touch probe 1 positive  |       |        | Pos.  |     |
| ----- | ----------------------- | ----- | ------ | ----- | --- |
| 60BAh | 00h                     | INT32 | ro Yes | –     | No  |
|       | edge                    |       |        | unit  |     |
z  60BBh: Touch probe 1 negative edge
This object provides the position value of the touch probe 1 at negative edge.
The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Touch probe 1 negative  |       |        | Pos.  |     |
| ----- | ----------------------- | ----- | ------ | ----- | --- |
| 60BBh | 00h                     | INT32 | ro Yes | –     | No  |
|       | edge                    |       |        | unit  |     |
z  60BCh: Touch probe 2 positive edge
This object provides the position value of the touch probe 2 at positive edge.
The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Touch probe 2 positive  |       |        | Pos.  |     |
| ----- | ----------------------- | ----- | ------ | ----- | --- |
| 60BCh | 00h                     | INT32 | ro Yes | –     | No  |
|       | edge                    |       |        | unit  |     |
z  60BDh: Touch probe 2 negative edge
This object provides the position value of the touch probe 2 at negative edge.
The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       | Touch probe 2 negative  |       |        | Pos.  |     |
| ----- | ----------------------- | ----- | ------ | ----- | --- |
| 60BDh | 00h                     | INT32 | ro Yes | –     | No  |
|       | edge                    |       |        | unit  |     |
109

## Page 110

Object Dictionary
z 60D5h: Touch probe 1 positive edge counter
This object provides a continuous counter that is incremented with each positive edge at touch probe 1.
The counter is only valid if the touch probe input is enabled (60B8h bit 0 = 1).
For single event measuring only the value of bit 0 is evaluated.
For continuous measuring the value is an unsigned 16-bit value with overflow.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Touch probe 1 positive
60D5h 00h UINT16 ro Yes – – No
edge counter
z 60D6h: Touch probe 1 negative edge counter
This object provides a continuous counter that is incremented with each negative edge at touch probe 1.
The counter is only valid if the touch probe input is enabled (60B8h bit 0 = 1).
For single event measuring only the value of bit 0 shall be evaluated.
For continuous measuring the value is an unsigned 16-bit value with overflow.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Touch probe 1 negative
60D6h 00h UINT16 ro Yes – – No
edge counter
z 60D7h: Touch probe 2 positive edge counter
This object provides a continuous counter that is incremented with each positive edge at touch probe 2.
The counter is only valid if the touch probe input is enabled (60B8h bit 8 = 1).
For single event measuring only the value of bit 0 shall be evaluated.
For continuous measuring the value is an unsigned 16-bit value with overflow.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Touch probe 2 positive
60D7h 00h UINT16 ro Yes – – No
edge counter
z 60D8h: Touch probe 2 negative edge counter
This object provides a continuous counter that is incremented with each negative edge at touch probe 2.
The counter is only valid if the touch probe input is enabled (60B8h bit 8 = 1).
For single event measuring only the value of bit 0 shall be evaluated.
For continuous measuring the value is an unsigned 16-bit value with overflow.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Touch probe 2 negative
60D8h 00h UINT16 ro Yes – – No
edge counter
110

## Page 111

Object Dictionary
z  60E3h: Supported homing methods
This object provides the supported homing methods of the drive.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Supported homing methods
Highest Sub-index 
| 00h | UINT8 | c No | 11  | – No |
| --- | ----- | ---- | --- | ---- |
Supported
1st supported homing 
| 01h | INT8 | ro No | 37  | – No |
| --- | ---- | ----- | --- | ---- |
method
2nd supported homing 
| 02h | INT8 | ro No | 35  | – No |
| --- | ---- | ----- | --- | ---- |
method
3rd supported homing 
| 03h | INT8 | ro No | 1   | – No |
| --- | ---- | ----- | --- | ---- |
method
4th supported homing 
| 04h | INT8 | ro No | 2   | – No |
| --- | ---- | ----- | --- | ---- |
method
5th supported homing 
| 05h | INT8 | ro No | 8   | – No |
| --- | ---- | ----- | --- | ---- |
60E3h method
6th supported homing 
| 06h | INT8 | ro No | 12  | – No |
| --- | ---- | ----- | --- | ---- |
method
7th supported homing 
| 07h | INT8 | ro No | 17  | – No |
| --- | ---- | ----- | --- | ---- |
method
8th supported homing 
| 08h | INT8 | ro No | 18  | – No |
| --- | ---- | ----- | --- | ---- |
method
9th supported homing 
| 09h | INT8 | ro No | 24  | – No |
| --- | ---- | ----- | --- | ---- |
method
10th supported homing 
| 0Ah | INT8 | ro No | 28  | – No |
| --- | ---- | ----- | --- | ---- |
method
11th supported homing 
| 0Bh | INT8 | ro No | –1  | – No |
| --- | ---- | ----- | --- | ---- |
method
111

## Page 112

Object Dictionary
z  60F2h: Positioning option code
The object is the positioning behavior in Profile Position mode.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 60F2h | 00h  | Positioning option code |     |     | UINT16 |     | rww | Yes | 0   |     | – No |
| ----- | ---- | ----------------------- | --- | --- | ------ | --- | --- | --- | --- | --- | ---- |
Data Description
| Bit 15   | 14       | 13 12                        | 11      | 10        | 9   | 8 7                       | 6           | 5 4 | 3   | 2   | 1 0 |
| -------- | -------- | ---------------------------- | ------- | --------- | --- | ------------------------- | ----------- | --- | --- | --- | --- |
| PUSH     |          | RSV [3]                      |         | IPOPT [4] |     | RADO                      |             | RRO |     | CIO | RO  |
| MSB      |          |                              |         |           |     |                           |             |     |     |     | LSB |
| Bit      | Notation |                              | Meaning |           |     |                           | Description |     |     |     |     |
| 0, 1     | RO       | Relative option              |         |           |     | Refer to following table. |             |     |     |     |     |
| 2, 3     | CIO      | Change immediately option    |         |           |     | Not supported.            |             |     |     |     |     |
| 4, 5     | RRO      | Request-response option      |         |           |     | Not supported.            |             |     |     |     |     |
| 6, 7     | RADO     | Rotary axis direction option |         |           |     | Refer to following table. |             |     |     |     |     |
| 8 to 11  | IPOPT    | IP option                    |         |           |     | Not supported.            |             |     |     |     |     |
| 12 to 14 | RSV      | Reserved                     |         |           |     | Reserved                  |             |     |     |     |     |
| 15       | PUSH     | Push-motion                  |         |           |     | Refer to following table. |             |     |     |     |     |
Bits in 
| Controlword  |     | Bits in Positioning option code (60F2h) |     |     |     |     |     |     |     |     |     |
| ------------ | --- | --------------------------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
(6040h)
Operation Mode
|           | Push- | Rotary axis  |     |                 |     |     |     |     |     |     |     |
| --------- | ----- | ------------ | --- | --------------- | --- | --- | --- | --- | --- | --- | --- |
| Abs / rel |       |              |     | Relative option |     |     |     |     |     |     |     |
motion direction option
| Bit 6  | Bit 15 | Bit 7 | Bit 6 | Bit 1 | Bit 0 |                                                  |     |     |     |     |     |
| ------ | ------ | ----- | ----- | ----- | ----- | ------------------------------------------------ | --- | --- | --- | --- | --- |
| 0      |        | 0 0   | 0     | X     | X     | Absolute positioning/Wrap absolute positioning * |     |     |     |     |     |
| 0      |        | 0 0   | 1     | X     | X     | Wrap reverse direction absolute positioning *    |     |     |     |     |     |
| 0      |        | 0 1   | 0     | X     | X     | Wrap forward direction absolute positioning *    |     |     |     |     |     |
| 0      |        | 0 1   | 1     | X     | X     | Wrap proximity positioning *                     |     |     |     |     |     |
Absolute positioning push-motion/Wrap absolute 
| 0   |     | 1 0 | 0   | X   | X   |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
push-motion *
| 0   |     | 1 0 | 1   | X   | X   | Wrap reverse direction push-motion * |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | ------------------------------------ | --- | --- | --- | --- | --- |
| 0   |     | 1 1 | 0   | X   | X   | Wrap forward direction push-motion * |     |     |     |     |     |
| 0   |     | 1 1 | 1   | X   | X   | Wrap proximity push-motion *         |     |     |     |     |     |
1 0 0 0 0 0 Incremental positioning (based on target position)
1 0 0 0 0 1 Incremental positioning (based on demand position)
1 0 0 0 1 0 Incremental positioning (based on actual position)
| 1   |     | 0 0 | 0   | 1   | 1   | Reserved |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | -------- | --- | --- | --- | --- | --- |
Incremental positioning push-motion (based on target 
| 1   |     | 1 0 | 0   | 0   | 0   |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
position)
Incremental positioning push-motion (based on 
| 1   |     | 1 0 | 0   | 0   | 1   |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
command position)
Incremental positioning push-motion (based on actual 
| 1   |     | 1 0 | 0   | 1   | 0   |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
position)
| 1   |     | 1 0 | 0   | 1   | 1   | Reserved |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | -------- | --- | --- | --- | --- | --- |
* To do this, Object 607Bh(Position range limit) must have set.
Bits marked by an X are irrelevant.
Refer to the following for details on the operation mode.
- OPERATING MANUAL BLV Series R Type Function Edition
112

## Page 113

Object Dictionary
z  60F4h: Following error actual value
This object provides the actual value of the following error.
The value is given in user-defined position units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
|       |      |     | Following error actual  |     |       |     |     |     |     |     | Pos.  |     |
| ----- | ---- | --- | ----------------------- | --- | ----- | --- | --- | --- | --- | --- | ----- | --- |
| 60F4h | 00h  |     |                         |     | INT32 |     | ro  | Yes |     | –   |       | No  |
|       |      |     | value                   |     |       |     |     |     |     |     | unit  |     |
z  60FDh: Digital inputs
This object provides the status of the driver and limit sensors.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 60FDh | 00h |     | Digital inputs |     | UINT32 |     | ro  | Yes |     | –   | –   | No  |
| ----- | --- | --- | -------------- | --- | ------ | --- | --- | --- | --- | --- | --- | --- |
Data Description
| Bit 31 | 30  | 29  | 28 27 | 26  | 25  | 24 23 | 22  | 21  | 20  | 19 18 | 17  | 16  |
| ------ | --- | --- | ----- | --- | --- | ----- | --- | --- | --- | ----- | --- | --- |
R-OUT [16]
| Bit 15 | 14       | 13  | 12 11   | 10      | 9   | 8 7 | 6   | 5           | 4   | 3 2     | 1   | 0   |
| ------ | -------- | --- | ------- | ------- | --- | --- | --- | ----------- | --- | ------- | --- | --- |
|        |          |     |         | RSV[12] |     |     |     |             |     | HWTO HS | PLS | NLS |
| MSB    |          |     |         |         |     |     |     |             |     |         |     | LSB |
| Bit    | Notation |     | Meaning |         |     |     |     | Description |     |         |     |     |
0: Negative limit switch not reached. 
| 0   | NLS | Negative limit switch |     |     |     |     |     |     |     |     |     |     |
| --- | --- | --------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: Negative limit switch reached.
0: Positive limit switch not reached. 
| 1   | PLS | Positive limit switch |     |     |     |     |     |     |     |     |     |     |
| --- | --- | --------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: Positive limit switch reached.
0: Home switch not reached. 
| 2   | HS  | Home switch |     |     |     |     |     |     |     |     |     |     |
| --- | --- | ----------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: Home switch reached.
0: Both the HWTO1 input and the HWTO2 input are not activated. 
| 3   | HWTO | HWTO input status |     |     |     |     |     |     |     |     |     |     |
| --- | ---- | ----------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
1: Either the HWTO1 input or the HWTO2 input is activated.
| 4 to 15 | RSV | Reserved |     |     | Reserved |     |     |     |     |     |     |     |
| ------- | --- | -------- | --- | --- | -------- | --- | --- | --- | --- | --- | --- | --- |
Output signals to R-OUT0 to R-OUT15 
| 16 to 31 | R-OUT | Remote output status |     |     |     |     |     |     |     |     |     |     |
| -------- | ----- | -------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
Details refer to following table.
Remote output status
| Bit | Name   |     | Default function |     |     | Bit |     | Name    | Default function |            |     |     |
| --- | ------ | --- | ---------------- | --- | --- | --- | --- | ------- | ---------------- | ---------- | --- | --- |
| 16  | R-OUT0 |     | SON-MON          |     |     | 24  |     | R-OUT8  |                  | SYS-BSY    |     |     |
| 17  | R-OUT1 |     | PLOOP-MON        |     |     | 25  |     | R-OUT9  |                  | IN-POS     |     |     |
| 18  | R-OUT2 |     | TRQ-LMTD         |     |     | 26  |     | R-OUT10 | RDY-HOME-OPE     |            |     |     |
| 19  | R-OUT3 |     | RDY-DD-OPE       |     |     | 27  |     | R-OUT11 | RDY-FWRV-OPE     |            |     |     |
| 20  | R-OUT4 |     | ABSPEN           |     |     | 28  |     | R-OUT12 |                  | RDY-SD-OPE |     |     |
| 21  | R-OUT5 |     | STOP_R           |     |     | 29  |     | R-OUT13 |                  | MOVE       |     |     |
| 22  | R-OUT6 |     | FREE_R           |     |     | 30  |     | R-OUT14 |                  | VA         |     |     |
| 23  | R-OUT7 |     | ALM-A            |     |     | 31  |     | R-OUT15 |                  | TLC        |     |     |
Refer to the following for details on the function assigned.
- OPERATING MANUAL BLV Series R Type Function Edition
113

## Page 114

Object Dictionary
z  60FEh: Digital outputs
This object controls the remote I/O.
If it arranges the function assignment, it can control the direct output signal.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
Digital outputs
Highest Sub-index 
|     | 00h |     |     | UINT8 |     | c   | No  | 1   |     | –   | No  |
| --- | --- | --- | --- | ----- | --- | --- | --- | --- | --- | --- | --- |
Supported
60FEh
0000 0000h  
|     |     | Digital output – physical  |     |        |     |     |     | to          |     |     |     |
| --- | --- | -------------------------- | --- | ------ | --- | --- | --- | ----------- | --- | --- | --- |
|     | 01h |                            |     | UINT32 |     | rww | Yes |             |     | –   | No  |
|     |     | outputs                    |     |        |     |     |     | FFFF FFFFh  |     |     |     |
(0000 0000h)
Data Description
| Bit 31 | 30  | 29  | 28 27 26 | 25 24 | 23  | 22  | 21 20 | 19  | 18  | 17  | 16  |
| ------ | --- | --- | -------- | ----- | --- | --- | ----- | --- | --- | --- | --- |
R-IN [16]
| Bit 15 | 14  | 13  | 12 11 10 | 9 8 | 7   | 6   | 5 4 | 3   | 2   | 1   | 0   |
| ------ | --- | --- | -------- | --- | --- | --- | --- | --- | --- | --- | --- |
RSV [16]
| MSB     |          |          |         |          |     |     |             |     |     |     | LSB |
| ------- | -------- | -------- | ------- | -------- | --- | --- | ----------- | --- | --- | --- | --- |
| Bit     | Notation |          | Meaning |          |     |     | Description |     |     |     |     |
| 0 to 15 | RSV      | Reserved |         | Reserved |     |     |             |     |     |     |     |
Input signals to R-IN0 to R-IN15 (Driver input 
| 16 to 31 | R-IN | Remote input signal |     |     |     |     |     |     |     |     |     |
| -------- | ---- | ------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
command (2nd)). Details refer to following table.
Remote input signal
| Bit |     | Name  | Default function |     |     | Bit | Name   |     | Default function |     |     |
| --- | --- | ----- | ---------------- | --- | --- | --- | ------ | --- | ---------------- | --- | --- |
| 16  |     | R-IN0 | S-ON             |     |     | 24  | R-IN8  |     | D-SEL0           |     |     |
| 17  |     | R-IN1 | PLOOP-MODE       |     |     | 25  | R-IN9  |     | D-SEL1           |     |     |
| 18  |     | R-IN2 | TRQ-LMT          |     |     | 26  | R-IN10 |     | D-SEL2           |     |     |
| 19  |     | R-IN3 | CLR              |     |     | 27  | R-IN11 |     | D-SEL3           |     |     |
| 20  |     | R-IN4 | QSTOP            |     |     | 28  | R-IN12 |     | D-SEL4           |     |     |
| 21  |     | R-IN5 | STOP             |     |     | 29  | R-IN13 |     | D-SEL5           |     |     |
| 22  |     | R-IN6 | FREE             |     |     | 30  | R-IN14 |     | D-SEL6           |     |     |
| 23  |     | R-IN7 | ALM-RST          |     |     | 31  | R-IN15 |     | D-SEL7           |     |     |
Refer to the following for details on the function assigned.
- OPERATING MANUAL BLV Series R Type Function Edition
z  60FFh: Target velocity (pv)
This object is the target velocity for Profile Velocity Mode.
The value is given in user-defined velocity units.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
–4,000,000 
Vel. 
| 60FFh | 00h | Target velocity (pv) |     | INT32 |     | rww | Yes |  to   |     |     | No  |
| ----- | --- | -------------------- | --- | ----- | --- | --- | --- | ----- | --- | --- | --- |
unit
4,000,000 (0)
114

## Page 115

Object Dictionary
z  6502h: Supported drive modes
This object provides information on the supported operation modes.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
0000 0025h
| 6502h | 00h Supported drive modes |     | UINT32 | ro  | Yes |     | – No |
| ----- | ------------------------- | --- | ------ | --- | --- | --- | ---- |
0000 002Dh*
Data Description
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21  | 20 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | --- | -------- | ----- |
RSV [16]
| Bit 15 | 14 13 12 | 11 10                 | 9 8            | 7 6    | 5   | 4 3 2             | 1 0   |
| ------ | -------- | --------------------- | -------------- | ------ | --- | ----------------- | ----- |
|        | RSV[5]   | CSTCA                 | CST CSV        | CSP IP | HM  | RSV TQ PV         | VL PP |
| MSB    |          |                       |                |        |     |                   | LSB   |
| Bit    | Notation |                       | Operation Mode |        |     | Definition        |       |
| 0      | PP       | Profile position mode |                |        |     | 1: Supported.     |       |
| 1      | VL       | Velocity mode         |                |        |     | 0: Not supported. |       |
| 2      | PV       | Profile velocity mode |                |        |     | 1: Supported.     |       |
0: Not supported. 
| 3   | TQ  | Profile torque mode |     |     |     |     |     |
| --- | --- | ------------------- | --- | --- | --- | --- | --- |
1: Supported.*
| 4   | RSV | Reserved                   |     |     |     | 0: Reserved       |     |
| --- | --- | -------------------------- | --- | --- | --- | ----------------- | --- |
| 5   | HM  | Homing mode                |     |     |     | 1: Supported.     |     |
| 6   | IP  | Interpolated position mode |     |     |     | 0: Not supported. |     |
| 7   | CSP | Cyclic sync position mode  |     |     |     | 0: Not supported. |     |
| 8   | CSV | Cyclic sync velocity mode  |     |     |     | 0: Not supported. |     |
| 9   | CST | Cyclic sync torque mode    |     |     |     | 0: Not supported. |     |
10 CSTCA Cyclic sync torque mode with commutation angle 0: Not supported.
| 11 to 31 | RSV | Reserved |     |     |     | 0: Reserved |     |
| -------- | --- | -------- | --- | --- | --- | ----------- | --- |
* It is effective for the driver version 4.00 or later.
115

## Page 116

Object Dictionary
z  67FEh: Version number
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 67FEh | 00h Version number |     | UINT32 | ro  | No  | 0004 0100h | – No |
| ----- | ------------------ | --- | ------ | --- | --- | ---------- | ---- |
Data Description
| Bit 31   | 30 29 28              | 27 26      | 25 24 | 23 22 | 21                    | 20 19 18 | 17 16 |
| -------- | --------------------- | ---------- | ----- | ----- | --------------------- | -------- | ----- |
|          | Reserved [8]          |            |       |       | Major version no. [8] |          |       |
| Bit 15   | 14 13 12              | 11 10      | 9 8   | 7 6   | 5                     | 4 3 2    | 1 0   |
|          | Minor version no. [8] |            |       |       | Sub version no. [8]   |          |       |
| MSB      |                       |            |       |       |                       |          | LSB   |
| Bit      | Notation              | Definition |       |       |                       |          |       |
| 0 to 7   | Sub version no.       |            | 0     |       |                       |          |       |
| 8 to 15  | Minor version no.     |            | 1     |       |                       |          |       |
| 16 to 23 | Major version no.     |            | 4     |       |                       |          |       |
| 24 to 31 | Reserved              |            | 0     |       |                       |          |       |
z  67FFh: Single device type
The object defines the type of each device within one drive unit and its functionality.
Index Sub-index Name Data type Access PDO Value (Default) Unit Save
| 67FFh | 00h Single device type |     | UINT32 | ro  | No  | 0002 0192h | – No |
| ----- | ---------------------- | --- | ------ | --- | --- | ---------- | ---- |
Data Description
| Bit 31 | 30 29 28 | 27 26 | 25 24 | 23 22 | 21  | 20 19 18 | 17 16 |
| ------ | -------- | ----- | ----- | ----- | --- | -------- | ----- |
Additional Information
| Bit 15 | 14 13 12 | 11 10 | 9 8 | 7 6 | 5   | 4 3 2 | 1 0 |
| ------ | -------- | ----- | --- | --- | --- | ----- | --- |
Device profile number
| MSB |     |     |     |     |     |     | LSB |
| --- | --- | --- | --- | --- | --- | --- | --- |
Additional Information: 2 (0002h) Servo Drive (Brushless motor driver)
Device profile number: 402 (0192h) DS402 drive profile
116

## Page 117

Appendix
9  Appendix
9.1  Object list
|           |                | Data  Acc- |               | Range       |             |           |
| --------- | -------------- | ---------- | ------------- | ----------- | ----------- | --------- |
| Index Sub | Name           |            | PDO           |             |             | Unit Save |
|           |                | type ess   |               |             |             |           |
|           |                |            | Default       | Lower Limit | Upper Limit |           |
| 1000h 00h | Device type    | UINT32 c   | No 0002 0192h | –           | –           | – No      |
| 1001h 00h | Error register | UINT8 ro   | No –          | –           | –           | – No      |
Pre-defined error field
| 00h | Number of errors | UINT8 ro | No 0  | 0   | 10  | – No |
| --- | ---------------- | -------- | ----- | --- | --- | ---- |
Standard error 
| 01h |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 1
Standard error 
| 02h |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 2
Standard error 
| 03h |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 3
Standard error 
| 04h |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 4
Standard error 
| 1003h 05h |     | UINT32 ro | No 0  | –   | –   | – No |
| --------- | --- | --------- | ----- | --- | --- | ---- |
field 5
Standard error 
| 06h |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 6
Standard error 
| 07h |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 7
Standard error 
| 08h |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 8
Standard error 
| 09h |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 9
Standard error 
| 0Ah |     | UINT32 ro | No 0  | –   | –   | – No |
| --- | --- | --------- | ----- | --- | --- | ---- |
field 10
COB-ID SYNC 
1005h 00h UINT32 rw No 0000 0080h 0000 0000h FFFF FFFFh – Yes
message
Communication 
| 1006h 00h |     | UINT32 rw | No 0  | 0   | 1,000,000  | – Yes |
| --------- | --- | --------- | ----- | --- | ---------- | ----- |
cycle period
BLVD-KRD
Manufacturer 
| 1008h 00h |                  | STRING c  | No        | –   | –       | – No   |
| --------- | ---------------- | --------- | --------- | --- | ------- | ------ |
|           | device name      |           | BLVD-KBRD |     |         |        |
|           | Manufacturer     |           | Hardware  |     |         |        |
| 1009h 00h |                  | STRING c  | No        | –   | –       | – No   |
|           | hardware version |           | version   |     |         |        |
|           | Manufacturer     |           | Software  |     |         |        |
| 100Ah 00h |                  | STRING c  | No        | –   | –       | – No   |
|           | software version |           | version   |     |         |        |
| 100Ch 00h | Guard time       | UINT16 rw | No 0      | 0   | 65,535  | ms Yes |
| 100Dh 00h | Life time factor | UINT8 rw  | No 0      | 0   | 255     | – Yes  |
Store parameters
Highest sub-index 
| 00h |     | UINT8 ro | No 2  | –   | –   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
supported
Save all 
| 1010h 01h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – No |
| --------- | --- | --------- | ------------- | ---------- | ---------- | ---- |
parameters
Save 
02h communication  UINT32 rw No 0000 0000h 0000 0000h FFFF FFFFh – No
parameters
117

## Page 118

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
Restore default parameters
Highest sub-index 
| 00h |     | UINT8 ro | No 2  | –   | –   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
supported
Restore all default 
| 01h              |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – No |
| ---------------- | --- | --------- | ------------- | ---------- | ---------- | ---- |
| 1011h parameters |     |           |               |            |            |      |
Restore 
communication 
| 02h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – No |
| --- | --- | --------- | ------------- | ---------- | ---------- | ---- |
default 
parameters
80h + 
1014h 00h COB-ID EMCY UINT32 rw No 0000 0000h FFFF FFFFh – Yes
Node-ID
Consumer heartbeat time
Highest sub-index 
| 00h             |     | UINT8 ro | No 1  | –   | –   | – No |
| --------------- | --- | -------- | ----- | --- | --- | ---- |
| 1016h supported |     |          |       |     |     |      |
Consumer 
| 01h |     | UINT32 rw | No 0000 0000h | 0000 0000h | 00FF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
heartbeat time
Producer 
| 1017h 00h |     | UINT16 rw | No 0  | 0   | 65,535  | ms Yes |
| --------- | --- | --------- | ----- | --- | ------- | ------ |
heartbeat time
Identity object
Highest sub-index 
| 00h |     | UINT8 ro | No 2  | –   | –   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
supported
| 01h Vender ID |     | UINT32 ro | No 0000 02BEh | –   | –   | – No |
| ------------- | --- | --------- | ------------- | --- | --- | ---- |
1018h
BLVD-KRD 
0000 13F7h
| 02h Product code |     | UINT32 ro | No  | –   | –   | – No |
| ---------------- | --- | --------- | --- | --- | --- | ---- |
BLVD-KBRD 
0000 1430h
SDO server parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 2  | –   | –   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
supported
1200h
| COB-ID client ->  |     |           | 600h +  |            |            |      |
| ----------------- | --- | --------- | ------- | ---------- | ---------- | ---- |
| 01h               |     | UINT32 ro | No      | 0000 0000h | FFFF FFFFh | – No |
| server (rx)       |     |           | Node-ID |            |            |      |
| COB-ID server ->  |     |           | 580h +  |            |            |      |
| 02h               |     | UINT32 ro | No      | 0000 0000h | FFFF FFFFh | – No |
| client (tx)       |     |           | Node-ID |            |            |      |
1st RPDO communication parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 2  | –   | –   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
supported
1400h
| COB-ID used by        |     |           | 200h +  |            |            |       |
| --------------------- | --- | --------- | ------- | ---------- | ---------- | ----- |
| 01h                   |     | UINT32 rw | No      | 0000 0000h | FFFF FFFFh | – Yes |
| RPDO                  |     |           | Node-ID |            |            |       |
| 02h Transmission type |     | UINT8 rw  | No 255  | 0          | 255        | – Yes |
2nd RPDO communication parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 2  | –   | –   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
supported
1401h
| COB-ID used by        |     |           | 300h +  |            |            |       |
| --------------------- | --- | --------- | ------- | ---------- | ---------- | ----- |
| 01h                   |     | UINT32 rw | No      | 0000 0000h | FFFF FFFFh | – Yes |
| RPDO                  |     |           | Node-ID |            |            |       |
| 02h Transmission type |     | UINT8 rw  | No 255  | 0          | 255        | – Yes |
3rd RPDO communication parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 2  | –   | –   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
supported
1402h
| COB-ID used by        |     |           | 400h +  |            |            |       |
| --------------------- | --- | --------- | ------- | ---------- | ---------- | ----- |
| 01h                   |     | UINT32 rw | No      | 0000 0000h | FFFF FFFFh | – Yes |
| RPDO                  |     |           | Node-ID |            |            |       |
| 02h Transmission type |     | UINT8 rw  | No 255  | 0          | 255        | – Yes |
118

## Page 119

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
4th RPDO communication parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 2 | –   | –   | – No |
| --- | --- | -------- | ---- | --- | --- | ---- |
supported
1403h
| COB-ID used by        |     |           | 500h +  |            |            |       |
| --------------------- | --- | --------- | ------- | ---------- | ---------- | ----- |
| 01h                   |     | UINT32 rw | No      | 0000 0000h | FFFF FFFFh | – Yes |
| RPDO                  |     |           | Node-ID |            |            |       |
| 02h Transmission type |     | UINT8 rw  | No 255  | 0          | 255        | – Yes |
1st RPDO mapping parameter
Number of 
mapped 
| 00h |     | UINT8 rw | No 1  | 0   | 4   | – Yes |
| --- | --- | -------- | ----- | --- | --- | ----- |
application 
objects in PDO
1st application 
| 01h          |     | UINT32 rw | No 6040 0010h | 0000 0000h | FFFF FFFFh | – Yes |
| ------------ | --- | --------- | ------------- | ---------- | ---------- | ----- |
| 1600h object |     |           |               |            |            |       |
2nd application 
| 02h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd application 
| 03h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th application 
| 04h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
2nd RPDO mapping parameter
Number of 
mapped 
| 00h |     | UINT8 rw | No 2  | 0   | 4   | – Yes |
| --- | --- | -------- | ----- | --- | --- | ----- |
application 
objects in PDO
1st application 
| 01h          |     | UINT32 rw | No 6040 0010h | 0000 0000h | FFFF FFFFh | – Yes |
| ------------ | --- | --------- | ------------- | ---------- | ---------- | ----- |
| 1601h object |     |           |               |            |            |       |
2nd application 
| 02h |     | UINT32 rw | No 6060 0008h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd application 
| 03h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th application 
| 04h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd RPDO mapping parameter
Number of 
mapped 
| 00h |     | UINT8 rw | No 2  | 0   | 4   | – Yes |
| --- | --- | -------- | ----- | --- | --- | ----- |
application 
objects in PDO
1st application 
| 01h          |     | UINT32 rw | No 6040 0010h | 0000 0000h | FFFF FFFFh | – Yes |
| ------------ | --- | --------- | ------------- | ---------- | ---------- | ----- |
| 1602h object |     |           |               |            |            |       |
2nd application 
| 02h |     | UINT32 rw | No 607A 0020h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd application 
| 03h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th application 
| 04h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
119

## Page 120

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
4th RPDO mapping parameter
Number of 
mapped 
| 00h |     | UINT8 rw | No 2  | 0   | 4   | – Yes |
| --- | --- | -------- | ----- | --- | --- | ----- |
application 
objects in PDO
1st application 
| 01h          |     | UINT32 rw | No 6040 0010h | 0000 0000h | FFFF FFFFh | – Yes |
| ------------ | --- | --------- | ------------- | ---------- | ---------- | ----- |
| 1603h object |     |           |               |            |            |       |
2nd application 
| 02h |     | UINT32 rw | No 60FF 0020h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd application 
| 03h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th application 
| 04h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
1st TPDO communication parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 5 | –   | –   | – No |
| --- | --- | -------- | ---- | --- | --- | ---- |
supported
| COB-ID used by  |     |           | 4000 0180h  |            |            |       |
| --------------- | --- | --------- | ----------- | ---------- | ---------- | ----- |
| 01h             |     | UINT32 rw | No          | 0000 0000h | FFFF FFFFh | – Yes |
| TPDO            |     |           | + Node-ID   |            |            |       |
1800h
| 02h Transmission type |     | UINT8 rw  | No 255  | 0   | 255     | – Yes      |
| --------------------- | --- | --------- | ------- | --- | ------- | ---------- |
| 03h Inhibit time      |     | UINT16 rw | No 50   | 0   | 65,535  | 100 μs Yes |
| 04h Reserved          |     | – –       | – –     | –   | –       | – –        |
| 05h Event timer       |     | UINT16 rw | No 0    | 0   | 65,535  | ms Yes     |
2nd TPDO communication parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 5 | –   | –   | – No |
| --- | --- | -------- | ---- | --- | --- | ---- |
supported
| COB-ID used by  |     |           | 4000 0280h  |            |            |       |
| --------------- | --- | --------- | ----------- | ---------- | ---------- | ----- |
| 01h             |     | UINT32 rw | No          | 0000 0000h | FFFF FFFFh | – Yes |
| TPDO            |     |           | + Node-ID   |            |            |       |
1801h
| 02h Transmission type |     | UINT8 rw  | No 255  | 0   | 255     | – Yes      |
| --------------------- | --- | --------- | ------- | --- | ------- | ---------- |
| 03h Inhibit time      |     | UINT16 rw | No 50   | 0   | 65,535  | 100 μs Yes |
| 04h Reserved          |     | - -       | - –     | –   | –       | – –        |
| 05h Event timer       |     | UINT16 rw | No 0    | 0   | 65,535  | ms Yes     |
3rd TPDO communication parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 5 | –   | –   | – No |
| --- | --- | -------- | ---- | --- | --- | ---- |
supported
| COB-ID used by  |     |           | 4000 0380h  |            |            |       |
| --------------- | --- | --------- | ----------- | ---------- | ---------- | ----- |
| 01h             |     | UINT32 rw | No          | 0000 0000h | FFFF FFFFh | – Yes |
| TPDO            |     |           | + Node-ID   |            |            |       |
1802h
| 02h Transmission type |     | UINT8 rw  | No 1   | 0   | 255     | – Yes      |
| --------------------- | --- | --------- | ------ | --- | ------- | ---------- |
| 03h Inhibit time      |     | UINT16 rw | No 50  | 0   | 65,535  | 100 μs Yes |
| 04h Reserved          |     | – –       | – –    | –   | –       | – –        |
| 05h Event timer       |     | UINT16 rw | No 0   | 0   | 65,535  | ms Yes     |
4th TPDO communication parameter
Highest sub-index 
| 00h |     | UINT8 ro | No 5 | –   | –   | – No |
| --- | --- | -------- | ---- | --- | --- | ---- |
supported
| COB-ID used by  |     |           | 4000 0480h  |            |            |       |
| --------------- | --- | --------- | ----------- | ---------- | ---------- | ----- |
| 01h             |     | UINT32 rw | No          | 0000 0000h | FFFF FFFFh | – Yes |
| TPDO            |     |           | + Node-ID   |            |            |       |
1803h
| 02h Transmission type |     | UINT8 rw  | No 1   | 0   | 255     | – Yes      |
| --------------------- | --- | --------- | ------ | --- | ------- | ---------- |
| 03h Inhibit time      |     | UINT16 rw | No 50  | 0   | 65,535  | 100 μs Yes |
| 04h Reserved          |     | – –       | – –    | –   | –       | – –        |
| 05h Event timer       |     | UINT16 rw | No 0   | 0   | 65,535  | ms Yes     |
120

## Page 121

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
1st TPDO mapping parameter
Number of 
mapped 
| 00h |     | UINT8 ro | No 1  | 0   | 4   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
application 
objects in TPDO
1st application 
| 01h          |     | UINT32 rw | No 6041 0010h | 0000 0000h | FFFF FFFFh | – Yes |
| ------------ | --- | --------- | ------------- | ---------- | ---------- | ----- |
| 1A00h object |     |           |               |            |            |       |
2nd application 
| 02h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd application 
| 03h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th application 
| 04h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
2nd TPDO mapping parameter
Number of 
mapped 
| 00h |     | UINT8 ro | No 2  | 0   | 4   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
application 
objects in TPDO
1st application 
| 01h          |     | UINT32 rw | No 6041 0010h | 0000 0000h | FFFF FFFFh | – Yes |
| ------------ | --- | --------- | ------------- | ---------- | ---------- | ----- |
| 1A01h object |     |           |               |            |            |       |
2nd application 
| 02h |     | UINT32 rw | No 6061 0008h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd application 
| 03h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th application 
| 04h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd TPDO mapping parameter
Number of 
mapped 
| 00h |     | UINT8 ro | No 2  | 0   | 4   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
application 
objects in TPDO
1st application 
| 01h          |     | UINT32 rw | No 6041 0010h | 0000 0000h | FFFF FFFFh | – Yes |
| ------------ | --- | --------- | ------------- | ---------- | ---------- | ----- |
| 1A02h object |     |           |               |            |            |       |
2nd application 
| 02h |     | UINT32 rw | No 6064 0020h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd application 
| 03h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th application 
| 04h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th TPDO mapping parameter
Number of 
mapped 
| 00h |     | UINT8 ro | No 2  | 0   | 4   | – No |
| --- | --- | -------- | ----- | --- | --- | ---- |
application 
objects in TPDO
1st application 
| 01h          |     | UINT32 rw | No 6041 0010h | 0000 0000h | FFFF FFFFh | – Yes |
| ------------ | --- | --------- | ------------- | ---------- | ---------- | ----- |
| 1A03h object |     |           |               |            |            |       |
2nd application 
| 02h |     | UINT32 rw | No 606C 0020h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
3rd application 
| 03h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
4th application 
| 04h |     | UINT32 rw | No 0000 0000h | 0000 0000h | FFFF FFFFh | – Yes |
| --- | --- | --------- | ------------- | ---------- | ---------- | ----- |
object
121

## Page 122

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
Direct data 
operation 
| 402Ch 00h |     | INT16 rww | Yes 0  | 0   | 255  | – No |
| --------- | --- | --------- | ------ | --- | ---- | ---- |
operation data 
number
Direct data 
| 402Dh 00h | operation  | UINT8 rww | Yes 0  | 0   | 255 | – No |
| --------- | ---------- | --------- | ------ | --- | --- | ---- |
operation type
Direct data 
Pos. 
402Eh 00h operation  INT32 rww Yes 0  –2,147,483,648 2,147,483,647  No
unit
position
Direct data 
Vel. 
402Fh 00h operation  INT32 rww Yes 0  –4,000,000  4,000,000  No
unit
operating velocity
|     | Direct data  |     |     |     |     | Acc.  |
| --- | ------------ | --- | --- | --- | --- | ----- |
4030h 00h operation  INT32 rww Yes 1,000  1  1,000,000,000  unit  No
|     | acceleration rate |     |     |     |     | (MS)  |
| --- | ----------------- | --- | --- | --- | --- | ----- |
|     | Direct data       |     |     |     |     | Acc.  |
4031h 00h operation  INT32 rww Yes 1,000  1  1,000,000,000  unit  No
|     | deceleration rate |     |     |     |     | (MS) |
| --- | ----------------- | --- | --- | --- | --- | ---- |
Direct data 
4032h 00h operation torque  INT16 rww Yes 10,000  0  10,000  0.1% No
limiting value
Direct data 
| 4033h 00h |     | INT32 rww | Yes 0  | –7  | 2,147,418,131 | – No |
| --------- | --- | --------- | ------ | --- | ------------- | ---- |
operation trigger
Direct data 
operation 
| 4034h 00h |     | UINT8 rww | Yes 0  | 0   | 1   | – No |
| --------- | --- | --------- | ------ | --- | --- | ---- |
forwarding 
destination
Driver input 
403Ah 00h UINT32 rww Yes 0000 0000h 0000 0000h FFFF FFFFh – No
command (2nd)
Driver input 
403Ch 00h command  UINT32 rww Yes 0000 0000h 0000 0000h FFFF FFFFh – No
(automatic OFF)
NET selection 
| 403Dh 00h |     | UINT32 rww | Yes 0  | 0   | 255  | – No |
| --------- | --- | ---------- | ------ | --- | ---- | ---- |
data number
Driver input 
403Eh 00h UINT32 rww Yes 0000 0000h 0000 0000h FFFF FFFFh – No
command
Driver output 
| 403Fh 00h |     | UINT32 ro | Yes – | –   | –   | – No |
| --------- | --- | --------- | ----- | --- | --- | ---- |
status
Target position 
Pos. 
| 404Bh 00h | (User-defined  | INT32 ro | Yes – | –   | –   | No  |
| --------- | -------------- | -------- | ----- | --- | --- | --- |
unit
position unit)
Demand position 
Pos. 
| 404Ch 00h | (User-defined  | INT32 ro | Yes – | –   | –   | No  |
| --------- | -------------- | -------- | ----- | --- | --- | --- |
unit
position unit)
Actual position 
Pos. 
| 404Dh 00h | (User-defined  | INT32 ro | Yes – | –   | –   | No  |
| --------- | -------------- | -------- | ----- | --- | --- | --- |
unit
position unit)
Target velocity 
Vel. 
| 404Eh 00h | (User-defined  | INT32 ro | Yes – | –   | –   | No  |
| --------- | -------------- | -------- | ----- | --- | --- | --- |
unit
velocity unit)
Demand velocity 
Vel. 
| 404Fh 00h | (User-defined  | INT32 ro | Yes – | –   | –   | No  |
| --------- | -------------- | -------- | ----- | --- | --- | --- |
unit
velocity unit)
122

## Page 123

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
Actual velocity 
Vel. 
| 4050h 00h | (User-defined  | INT32 ro | Yes – | –   | –   | No  |
| --------- | -------------- | -------- | ----- | --- | --- | --- |
unit
velocity unit)
Present 
| 4056h 00h | communication  | UINT8 ro | Yes – | –   | –   | – No |
| --------- | -------------- | -------- | ----- | --- | --- | ---- |
error
| 406Bh 00h | Torque monitor | INT16 ro | Yes – | –   | –   | 0.1% No |
| --------- | -------------- | -------- | ----- | --- | --- | ------- |
Load factor 
| 406Ch 00h |     | INT32 ro | Yes – | –   | –   | 0.1% No |
| --------- | --- | -------- | ----- | --- | --- | ------- |
monitor
Cumulative load 
| 406Dh 00h |     | UINT32 ro | Yes – | –   | –   | – No |
| --------- | --- | --------- | ----- | --- | --- | ---- |
monitor
| 4070h 00h | Next data number | INT16 ro | Yes – | –   | –   | – No |
| --------- | ---------------- | -------- | ----- | --- | --- | ---- |
Loop origin data 
| 4071h 00h |     | INT16 ro | Yes – | –   | –   | – No |
| --------- | --- | -------- | ----- | --- | --- | ---- |
number
| 4072h 00h | Loop count | UINT32 ro | Yes – | –   | –   | – No |
| --------- | ---------- | --------- | ----- | --- | --- | ---- |
Pos. 
| 4073h 00h | Position deviation | INT32 ro | Yes – | –   | –   | No  |
| --------- | ------------------ | -------- | ----- | --- | --- | --- |
unit
Vel. 
| 4075h 00h | Speed deviation | INT32 ro | Yes – | –   | –   | No  |
| --------- | --------------- | -------- | ----- | --- | --- | --- |
unit
0.1 
| 407Ah 00h | Tripmeter 1 | INT32 ro | Yes – | –   | –   | No  |
| --------- | ----------- | -------- | ----- | --- | --- | --- |
krev
Information 
| 407Bh 00h |     | UINT32 ro | Yes – | –   | –   | – No |
| --------- | --- | --------- | ----- | --- | --- | ---- |
status 1
Driver 
| 407Ch 00h |     | INT16 ro | Yes – | –   | –   | 0.1°C No |
| --------- | --- | -------- | ----- | --- | --- | -------- |
temperature
Motor 
| 407Dh 00h |     | INT16 ro | Yes – | –   | –   | 0.1°C No |
| --------- | --- | -------- | ----- | --- | --- | -------- |
temperature
0.1 
| 407Eh 00h | Odometer | UINT32 ro | Yes – | –   | –   | No  |
| --------- | -------- | --------- | ----- | --- | --- | --- |
krev
0.1 
| 407Fh 00h | Tripmeter 0 | UINT32 ro | Yes – | –   | –   | No  |
| --------- | ----------- | --------- | ----- | --- | --- | --- |
krev
|           | Main power     |          |       |     |     | 0.001  |
| --------- | -------------- | -------- | ----- | --- | --- | ------ |
| 409Bh 00h |                | INT32 ro | Yes – | –   | –   | No     |
|           | supply current |          |       |     |     | A      |
Power 
| 409Ch 00h |     | UINT32 ro | Yes – | –   | –   | 0.1W No |
| --------- | --- | --------- | ----- | --- | --- | ------- |
consumption
|           | Energy      |           |       |     |     | 0.001  |
| --------- | ----------- | --------- | ----- | --- | --- | ------ |
| 409Dh 00h |             | UINT32 ro | Yes – | –   | –   | No     |
|           | consumption |           |       |     |     | Wh     |
User energy 
| 409Eh 00h |     | UINT32 ro | Yes – | –   | –   | Wh No |
| --------- | --- | --------- | ----- | --- | --- | ----- |
consumption
Total energy 
| 409Fh 00h |     | UINT32 ro | Yes – | –   | –   | Wh No |
| --------- | --- | --------- | ----- | --- | --- | ----- |
consumption
| 40A1h 00h | Total uptime     | UINT32 ro | Yes – | –   | –   | min. No |
| --------- | ---------------- | --------- | ----- | --- | --- | ------- |
| 40A2h 00h | Number of boots  | UINT32 ro | Yes – | –   | –   | – No    |
| 40A3h 00h | Inverter voltage | INT16 ro  | Yes – | –   | –   | 0.1V No |
Main power 
| 40A4h 00h |     | INT16 ro | Yes – | –   | –   | 0.1V No |
| --------- | --- | -------- | ----- | --- | --- | ------- |
supply voltage
Continuous 
| 40A9h 00h |     | UINT32 ro | Yes – | –   | –   | ms No |
| --------- | --- | --------- | ----- | --- | --- | ----- |
uptime
RS-485 
communication 
| 40AAh 00h |     | UINT32 ro | Yes – | –   | –   | – No |
| --------- | --- | --------- | ----- | --- | --- | ---- |
reception byte 
counter
123

## Page 124

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
RS-485 
communication 
| 40ABh 00h |     | UINT32 ro | Yes – | –   | –   | – No |
| --------- | --- | --------- | ----- | --- | --- | ---- |
transmission byte 
counter
| 40C0h 00h | Alarm reset | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | ----------- | --------- | ------ | --- | --- | ---- |
Clear alarm 
| 40C2h 00h |     | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | --- | --------- | ------ | --- | --- | ---- |
history
P-PRESET 
| 40C5h 00h |     | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | --- | --------- | ------ | --- | --- | ---- |
execution
| 40C6h 00h | Configuration | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | ------------- | --------- | ------ | --- | --- | ---- |
Clear latch 
| 40CDh 00h |     | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | --- | --------- | ------ | --- | --- | ---- |
information
Clear sequence 
| 40CEh 00h |     | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | --- | --------- | ------ | --- | --- | ---- |
history
| 40D0h 00h | Clear ETO         | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | ----------------- | --------- | ------ | --- | --- | ---- |
| 40D1h 00h | ZSG-PRESET        | UINT8 rww | Yes 0  | 0   | 2   | – No |
| 40D2h 00h | Clear ZSG-PRESET  | UINT8 rww | Yes 0  | 0   | 2   | – No |
| 40D3h 00h | Clear information | UINT8 rww | Yes 0  | 0   | 2   | – No |
Clear user energy 
| 40D6h 00h |     | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | --- | --------- | ------ | --- | --- | ---- |
consumption
| 40D7h 00h | Clear tripmeter 0 | UINT8 rww | Yes 0  | 0   | 2   | – No |
| --------- | ----------------- | --------- | ------ | --- | --- | ---- |
| 40D8h 00h | Clear tripmeter 1 | UINT8 rww | Yes 0  | 0   | 2   | – No |
Permission of 
absolute 
positioning 
| 4148h 00h |     | UINT8 rww | Yes 0  | 0   | 1   | – No |
| --------- | --- | --------- | ------ | --- | --- | ---- |
without setting 
absolute 
coordinates
JOG/HOME 
| 415Fh 00h |     | UINT16 rww | Yes 10,000  | 0   | 10,000  | 0.1% Yes |
| --------- | --- | ---------- | ----------- | --- | ------- | -------- |
Torque limit value
(HOME) Homing 
| 4160h 00h |     | UINT8 rww | Yes 1  | 0   | 3   | – Yes |
| --------- | --- | --------- | ------ | --- | --- | ----- |
mode
|           | (HOME) Starting  |            |         |     |            | Vel.  |
| --------- | ---------------- | ---------- | ------- | --- | ---------- | ----- |
| 4163h 00h |                  | UINT32 rww | Yes 30  | 1   | 4,000,000  | Yes   |
|           | velocity         |            |         |     |            | unit  |
(HOME) Backward 
Pos. 
4169h 00h steps in 2 sensor  UINT32 rww Yes 18,000  0  8,388,607  Yes
unit
homeseeking
Stopping timeout 
| 4186h 00h | at alarm  | UINT16 rww | Yes 3,000  | 0   | 10,000  | ms Yes |
| --------- | --------- | ---------- | ---------- | --- | ------- | ------ |
generation
| 41CAh 00h | WRAP setting     | UINT8 rww  | Yes 1      | 1   | 2              | – Yes |
| --------- | ---------------- | ---------- | ---------- | --- | -------------- | ----- |
|           | Custom stopping  |            |            |     |                | Acc.  |
| 4735h 00h |                  | UINT32 rww | Yes 1,000  | 1   | 1,000,000,000  | Yes   |
|           | rate             |            |            |     |                | unit  |
Custom stopping 
| 4736h 00h |     | UINT32 rww | Yes 1,000  | 1   | 1,000,000,000  | ms Yes |
| --------- | --- | ---------- | ---------- | --- | -------------- | ------ |
time
| 603Fh 00h | Error code  | UINT16 ro  | Yes –     | –     | –     | – No |
| --------- | ----------- | ---------- | --------- | ----- | ----- | ---- |
| 6040h 00h | Controlword | UINT16 rww | Yes 0004h | 0000h | FFFFh | – No |
| 6041h 00h | Statusword  | UINT16 ro  | Yes –     | –     | –     | – No |
Quick stop option 
| 605Ah 00h |     | INT16 rw | No 2  | –3  | 6   | – Yes |
| --------- | --- | -------- | ----- | --- | --- | ----- |
code
Shutdown option 
| 605Bh 00h |     | INT16 rw | No 0  | 0   | 1   | – Yes |
| --------- | --- | -------- | ----- | --- | --- | ----- |
code
124

## Page 125

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
Disable operation 
| 605Ch 00h |     | INT16 rw | No 1  | 0   | 1   | – Yes |
| --------- | --- | -------- | ----- | --- | --- | ----- |
option code
| 605Dh 00h | Halt option code | INT16 rw | No 1  | –3  | 2   | – Yes |
| --------- | ---------------- | -------- | ----- | --- | --- | ----- |
Fault reaction 
| 605Eh 00h |     | INT16 rw | No 2  | 0   | 2   | – Yes |
| --------- | --- | -------- | ----- | --- | --- | ----- |
option code
Modes of 
| 6060h 00h |     | INT8 rww | Yes 3  | 0   | 6   | – Yes |
| --------- | --- | -------- | ------ | --- | --- | ----- |
operation
Modes of 
| 6061h 00h |     | INT8 ro | Yes – | –   | –   | – No |
| --------- | --- | ------- | ----- | --- | --- | ---- |
operation display
|           | Position demand  |            |              |     |             | Pos.  |
| --------- | ---------------- | ---------- | ------------ | --- | ----------- | ----- |
| 6062h 00h |                  | INT32 ro   | Yes –        | –   | –           | No    |
|           | value            |            |              |     |             | unit  |
|           | Position actual  |            |              |     |             | Pos.  |
| 6064h 00h |                  | INT32 ro   | Yes –        | –   | –           | No    |
|           | value            |            |              |     |             | unit  |
|           | Following error  |            |              |     |             | Pos.  |
| 6065h 00h |                  | UINT32 rww | Yes 100,800  | 0   | 10,000,000  | Yes   |
|           | window           |            |              |     |             | unit  |
Pos. 
| 6067h 00h | Position window | UINT32 rww | Yes 18  | 0   | 65,535  | Yes |
| --------- | --------------- | ---------- | ------- | --- | ------- | --- |
unit
|           | Velocity demand  |          |       |     |     | Vel.  |
| --------- | ---------------- | -------- | ----- | --- | --- | ----- |
| 606Bh 00h |                  | INT32 ro | Yes – | –   | –   | No    |
|           | value            |          |       |     |     | unit  |
|           | Velocity actual  |          |       |     |     | Vel.  |
| 606Ch 00h |                  | INT32 ro | Yes – | –   | –   | No    |
|           | value            |          |       |     |     | unit  |
Vel. 
| 606Dh 00h | Velocity window | UINT16 rww | Yes 15  | 1   | 65,535  | Yes |
| --------- | --------------- | ---------- | ------- | --- | ------- | --- |
unit
Vel. 
606Fh 00h Velocity threshold UINT16 rww Yes 15  1  65,535  Yes
unit
| 6071h 00h | Target torque | INT16 rww | Yes -1,000 | 0   | 1,000 | 0.1% No |
| --------- | ------------- | --------- | ---------- | --- | ----- | ------- |
6072h 00h Max torque UINT16 rww Yes 10,000  0  10,000  0.1% Yes
| 6074h 00h | Torque demand | INT16 ro | Yes – | –   | –   | 0.1% No |
| --------- | ------------- | -------- | ----- | --- | --- | ------- |
Torque actual 
| 6077h 00h |     | INT16 ro | Yes – | –   | –   | 0.1% No |
| --------- | --- | -------- | ----- | --- | --- | ------- |
value
Pos. 
607Ah 00h Target position INT32 rww Yes 0  –2,147,483,648  2,147,483,647  No
unit
Position range limit
Highest Sub-
| 00h |     | UINT8 c | No 2  | –   | –   | – No |
| --- | --- | ------- | ----- | --- | --- | ---- |
index Supported
607Bh
|     | Min position  |           |        |                 |                | Pos.  |
| --- | ------------- | --------- | ------ | --------------- | -------------- | ----- |
| 01h |               | INT32 rww | Yes 0  | –2,147,483,648  | 0              | Yes   |
|     | range limit   |           |        |                 |                | unit  |
|     | Max position  |           |        |                 |                | Pos.  |
| 02h |               | INT32 rww | Yes 0  | 0               | 2,147,483,647  | Yes   |
|     | range limit   |           |        |                 |                | unit  |
Pos. 
607Ch 00h Home offset INT32 rww Yes 0  –2,147,483,648  2,147,483,647  Yes
unit
Software position limit 
Highest Sub-
| 00h |     | UINT8 c | No 2  | –   | –   | – No |
| --- | --- | ------- | ----- | --- | --- | ---- |
index Supported
| 607Dh |     |     |     |     |     | Pos.  |
| ----- | --- | --- | --- | --- | --- | ----- |
01h Min position limit INT32 rww Yes 0  –2,147,483,648  2,147,483,647  Yes
unit
Pos. 
02h Max position limit INT32 rww Yes 0  –2,147,483,648  2,147,483,647  Yes
unit
Vel. 
6081h 00h Profile velocity UINT32 rww Yes 1  1  4,000,000  Yes
unit
Vel. 
| 6082h 00h | End velocity | UINT32 rww | Yes 0  | 0   | 4,000,000  | Yes |
| --------- | ------------ | ---------- | ------ | --- | ---------- | --- |
unit
125

## Page 126

Appendix
|           |              | Data  Acc- |            | Range       |                |           |
| --------- | ------------ | ---------- | ---------- | ----------- | -------------- | --------- |
| Index Sub | Name         |            | PDO        |             |                | Unit Save |
|           |              | type ess   | Default    | Lower Limit | Upper Limit    |           |
|           | Profile      |            |            |             |                | Acc.      |
| 6083h 00h |              | UINT32 rww | Yes 1,000  | 1           | 1,000,000,000  | Yes       |
|           | acceleration |            |            |             |                | unit      |
|           | Profile      |            |            |             |                | Acc.      |
| 6084h 00h |              | UINT32 rww | Yes 1,000  | 1           | 1,000,000,000  | Yes       |
|           | deceleration |            |            |             |                | unit      |
|           | Quick stop   |            |            |             |                | Acc.      |
| 6085h 00h |              | UINT32 rww | Yes 1,000  | 1           | 1,000,000,000  | Yes       |
|           | deceleration |            |            |             |                | unit      |
6087h 00h Torque slope UINT32 rww Yes 0 0 1,000,000 0.1%/s Yes
Position encoder resolution
Highest Sub-
| 00h |     | UINT8 c | No 2  | –   | –   | – No |
| --- | --- | ------- | ----- | --- | --- | ---- |
index Supported
608Fh
Encoder 
| 01h |     | UINT32 rww | Yes 36,000  | 1   | 65,535  | – Yes |
| --- | --- | ---------- | ----------- | --- | ------- | ----- |
increments
| 02h | Motor revolutions | UINT32 rww | Yes 1  | 1   | 65,535  | – Yes |
| --- | ----------------- | ---------- | ------ | --- | ------- | ----- |
Gear ratio
Highest sub-index 
| 00h       |                   | UINT8 c   | No 2    | –   | –      | – No  |
| --------- | ----------------- | --------- | ------- | --- | ------ | ----- |
| 6091h     | supported         |           |         |     |        |       |
| 01h       | Motor revolutions | UINT32 rw | No 1    | 1   | 1,000  | – Yes |
| 02h       | Shaft revolutions | UINT32 rw | No 1    | 1   | 1,000  | – Yes |
| 6098h 00h | Homing method     | INT8 rww  | Yes 37  | –1  | 37     | – Yes |
Homing speeds
Highest sub-index 
| 00h |     | UINT8 c | No 2  | –   | –   | – No |
| --- | --- | ------- | ----- | --- | --- | ---- |
supported
| 6099h     | Speed during      |            |            |     |                | Vel.  |
| --------- | ----------------- | ---------- | ---------- | --- | -------------- | ----- |
| 01h       |                   | UINT32 rww | Yes 60     | 1   | 4,000,000      | Yes   |
|           | search for switch |            |            |     |                | unit  |
|           | Speed during      |            |            |     |                | Vel.  |
| 02h       |                   | UINT32 rww | Yes 30     | 1   | 4,000,000      | Yes   |
|           | search for zero   |            |            |     |                | unit  |
|           | Homing            |            |            |     |                | Acc.  |
| 609Ah 00h |                   | UINT32 rww | Yes 1,000  | 1   | 1,000,000,000  | Yes   |
|           | acceleration      |            |            |     |                | unit  |
| 60A8h 00h | SI unit position  | UINT32 rw  | Yes –      | –   | –              | – No  |
| 60A9h 00h | SI unit velocity  | UINT32 rw  | Yes –      | –   | –              | – No  |
Touch probe 
| 60B8h 00h |     | UINT16 rww | Yes 0000h | 0000h | FFFFh | – Yes |
| --------- | --- | ---------- | --------- | ----- | ----- | ----- |
function
Touch probe 
| 60B9h 00h |     | UINT16 ro | Yes – | –   | –   | – No |
| --------- | --- | --------- | ----- | --- | --- | ---- |
status
|           | Touch probe 1  |          |       |     |     | Pos.  |
| --------- | -------------- | -------- | ----- | --- | --- | ----- |
| 60BAh 00h |                | INT32 ro | Yes – | –   | –   | No    |
|           | positive edge  |          |       |     |     | unit  |
|           | Touch probe 1  |          |       |     |     | Pos.  |
| 60BBh 00h |                | INT32 ro | Yes – | –   | –   | No    |
|           | negative edge  |          |       |     |     | unit  |
|           | Touch probe 2  |          |       |     |     | Pos.  |
| 60BCh 00h |                | INT32 ro | Yes – | –   | –   | No    |
|           | positive edge  |          |       |     |     | unit  |
|           | Touch probe 2  |          |       |     |     | Pos.  |
| 60BDh 00h |                | INT32 ro | Yes – | –   | –   | No    |
|           | negative edge  |          |       |     |     | unit  |
Touch probe 1 
| 60D5h 00h | positive edge  | UINT16 ro | Yes – | –   | –   | – No |
| --------- | -------------- | --------- | ----- | --- | --- | ---- |
counter
Touch probe 1 
| 60D6h 00h | negative edge  | UINT16 ro | Yes – | –   | –   | – No |
| --------- | -------------- | --------- | ----- | --- | --- | ---- |
counter
Touch probe 2 
| 60D7h 00h | positive edge  | UINT16 ro | Yes – | –   | –   | – No |
| --------- | -------------- | --------- | ----- | --- | --- | ---- |
counter
126

## Page 127

Appendix
|           |      | Data  Acc- |         | Range       |             |           |
| --------- | ---- | ---------- | ------- | ----------- | ----------- | --------- |
| Index Sub | Name |            | PDO     |             |             | Unit Save |
|           |      | type ess   | Default | Lower Limit | Upper Limit |           |
Touch probe 2 
| 60D8h 00h | negative edge  | UINT16 ro | Yes – | –   | –   | – No |
| --------- | -------------- | --------- | ----- | --- | --- | ---- |
counter
Supported homing methods
Highest Sub-
| 00h |     | UINT8 c | No 11  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
index Supported
1st supported 
| 01h |     | INT8 ro | No 37  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
homing method
2nd supported 
| 02h |     | INT8 ro | No 35  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
homing method
3rd supported 
| 03h |     | INT8 ro | No 1  | –   | –   | – No |
| --- | --- | ------- | ----- | --- | --- | ---- |
homing method
4th supported 
| 04h |     | INT8 ro | No 2  | –   | –   | – No |
| --- | --- | ------- | ----- | --- | --- | ---- |
homing method
5th supported 
| 05h   |               | INT8 ro | No 8  | –   | –   | – No |
| ----- | ------------- | ------- | ----- | --- | --- | ---- |
| 60E3h | homing method |         |       |     |     |      |
6th supported 
| 06h |     | INT8 ro | No 12  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
homing method
7th supported 
| 07h |     | INT8 ro | No 17  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
homing method
8th supported 
| 08h |     | INT8 ro | No 18  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
homing method
9th supported 
| 09h |     | INT8 ro | No 24  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
homing method
10th supported 
| 0Ah |     | INT8 ro | No 28  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
homing method
11th supported 
| 0Bh |     | INT8 ro | No –1  | –   | –   | – No |
| --- | --- | ------- | ------ | --- | --- | ---- |
homing method
Positioning 
| 60F2h 00h  |     | UINT16 rww | Yes 0  | 0000h | FFFFh | – No |
| ---------- | --- | ---------- | ------ | ----- | ----- | ---- |
option code
|            | Following error  |           |       |     |     | Pos.  |
| ---------- | ---------------- | --------- | ----- | --- | --- | ----- |
| 60F4h 00h  |                  | INT32 ro  | Yes – | –   | –   | No    |
|            | actual value     |           |       |     |     | unit  |
| 60FDh 00h  | Digital inputs   | UINT32 ro | Yes – | –   | –   | – No  |
Digital output
Highest Sub-
| 00h   |                 | UINT8 c | No 1  | –   | –   | – No |
| ----- | --------------- | ------- | ----- | --- | --- | ---- |
| 60FEh | index Supported |         |       |     |     |      |
Digital output-
| 01h |     | UINT32 rww | Yes 0000 0000h | 0000 0000h | FFFF FFFFh | – No |
| --- | --- | ---------- | -------------- | ---------- | ---------- | ---- |
physical outputs
|           | Target velocity  |           |        |             |            | Vel.  |
| --------- | ---------------- | --------- | ------ | ----------- | ---------- | ----- |
| 60FFh 00h |                  | INT32 rww | Yes 0  | –4,000,000  | 4,000,000  | No    |
|           | (pv)             |           |        |             |            | unit  |
Supported drive 
| 6502h 00h |     | UINT32 ro | Yes 0000 0025h | –   | –   | – No |
| --------- | --- | --------- | -------------- | --- | --- | ---- |
modes
| 67FEh 00h | Version number | UINT32 ro | No 0004 0100h | –   | –   | – No |
| --------- | -------------- | --------- | ------------- | --- | --- | ---- |
67FFh 00h Single device type UINT32 ro No 0002 0192h – – – No
127

## Page 128

Appendix
9.2 Specifications
In conformance with ISO 11898
Electrical characteristics
Use the CAN-Bus cable.
Communication protocol CANopen
Communication profile In conformance with CiA DS301 Version 4.2.0
Device profile In conformance with CiA DSP402 Version 4.0.0
Node ID 1 to 127
Bitrate Selectable from 1000, 800, 500(default), 250, 125, 50, 20, and 10 kbps
Maximum bus length 25 m (82 ft.) [maximum bus length at 1 Mbps]
NMT (Network Management)
SDO (Service Data Object: 1 SDO server)
Communication objects PDO (Process Data Object: 4 Receive-PDO, 4 Transmit-PDO)
EMCY (Emergency Object)
SYNC (Synchronization Object)
Profile velocity mode (pv)
Profile position mode (pp)
Operation modes
Profile torque mode (tq)*
Homing mode (hm)
* It is effective for the driver version 4.00 or later.
128

## Page 129

Appendix
129

## Page 130

• Please contact your nearest Oriental Motor office for further information.
Technical Support Tel:800-468-3982 Singapore Korea
8:30am EST to 5:00pm PST (M-F) Tel:1800-842-0280 Tel:080-777-2042
Schiessstraße 44, 40549 Düsseldorf, Germany Tel:1800-806-161 4-8-1 Higashiueno, Taito-ku, Tokyo
Technical Support Tel:00 800/22 55 66 22 110-8536 Japan
Tel:+81-3-6744-0361
Tel:1800-888-881
www.orientalmotor.co.jp/ja
Unit 5 Faraday Office Park, Rankine Road,
Basingstoke, Hampshire RG24 8QB UK
Tel:1800-120-1995 (For English)
Tel:+44-1256347090
1800-121-4149 (For Hindi)
Tel:+33-1 47 86 97 50
Tel:0800-060708
Tel:+39-02-93906347
Tel:400-820-6516
