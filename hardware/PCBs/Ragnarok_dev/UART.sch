EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 5200 1800 0    50   ~ 0
RX
Text Notes 5200 2000 0    50   ~ 0
TX\n\n
$Comp
L Connector:Conn_01x05_Female J?
U 1 1 5E2FBB26
P 5200 1900
AR Path="/5E201923/5E2FBB26" Ref="J?"  Part="1" 
AR Path="/5E2FBB26" Ref="J?"  Part="1" 
AR Path="/5E2F29D1/5E2FBB26" Ref="J3"  Part="1" 
F 0 "J3" H 5100 2350 50  0000 L CNN
F 1 "SPS30" H 5000 2250 50  0000 L CNN
F 2 "" H 5200 1900 50  0001 C CNN
F 3 "~" H 5200 1900 50  0001 C CNN
	1    5200 1900
	1    0    0    -1  
$EndComp
NoConn ~ 5000 2000
$Comp
L Ragnarok_dev-rescue:ublox_SAM-M8Q-RF_GPS U?
U 1 1 5E2FE35E
P 6000 6700
AR Path="/5E2FE35E" Ref="U?"  Part="1" 
AR Path="/5E2F29D1/5E2FE35E" Ref="U1"  Part="1" 
F 0 "U1" H 5850 6150 50  0000 C CNN
F 1 "ublox_SAM-M8Q" H 5600 6250 50  0000 C CNN
F 2 "RF_Module:ublox_SAM-M8Q_HandSolder" H 6550 6250 50  0001 C CNN
F 3 "https://www.u-blox.com/sites/default/files/SAM-M8Q_DataSheet_%28UBX-16012619%29.pdf" H 6000 6700 50  0001 C CNN
	1    6000 6700
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5E2FE36A
P 6200 6100
AR Path="/5E2FE36A" Ref="C?"  Part="1" 
AR Path="/5E2F29D1/5E2FE36A" Ref="C36"  Part="1" 
F 0 "C36" V 6100 6000 50  0000 C CNN
F 1 "0.1uF" V 6050 6000 50  0000 C CNN
F 2 "" H 6200 6100 50  0001 C CNN
F 3 "~" H 6200 6100 50  0001 C CNN
	1    6200 6100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 5E300D95
P 4900 2200
F 0 "#PWR0124" H 4900 1950 50  0001 C CNN
F 1 "GND" H 4905 2027 50  0000 C CNN
F 2 "" H 4900 2200 50  0001 C CNN
F 3 "" H 4900 2200 50  0001 C CNN
	1    4900 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2100 4900 2100
Wire Wire Line
	4900 2100 4900 2200
$Comp
L power:+5V #PWR0128
U 1 1 5E30193C
P 4900 1600
F 0 "#PWR0128" H 4900 1450 50  0001 C CNN
F 1 "+5V" H 4915 1773 50  0000 C CNN
F 2 "" H 4900 1600 50  0001 C CNN
F 3 "" H 4900 1600 50  0001 C CNN
	1    4900 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 1600 4900 1700
Wire Wire Line
	4900 1700 5000 1700
$Comp
L XBP9B-DMST-002:XBP9B-DMST-002 U2
U 1 1 5E3927AC
P 6100 3800
F 0 "U2" H 6100 4867 50  0000 C CNN
F 1 "XBP9B-DMST-002" H 6100 4776 50  0000 C CNN
F 2 "xbee:DIP2200W51P200L3294H279Q20P" H 6100 3800 50  0001 L BNN
F 3 "Digi International" H 6100 3800 50  0001 L BNN
	1    6100 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5E39A39D
P 6200 5950
AR Path="/5E39A39D" Ref="C?"  Part="1" 
AR Path="/5E2F29D1/5E39A39D" Ref="C35"  Part="1" 
F 0 "C35" V 6250 5850 50  0000 C CNN
F 1 "0.1uF" V 6350 5900 50  0000 C CNN
F 2 "" H 6200 5950 50  0001 C CNN
F 3 "~" H 6200 5950 50  0001 C CNN
	1    6200 5950
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0129
U 1 1 5E39B062
P 6100 5850
F 0 "#PWR0129" H 6100 5700 50  0001 C CNN
F 1 "+3.3V" H 6115 6023 50  0000 C CNN
F 2 "" H 6100 5850 50  0001 C CNN
F 3 "" H 6100 5850 50  0001 C CNN
	1    6100 5850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6100 5850 6100 5950
Connection ~ 6100 5950
Wire Wire Line
	6100 5950 6100 6100
Connection ~ 6100 6100
Wire Wire Line
	6100 6100 6100 6200
Wire Wire Line
	6300 5950 6400 5950
Wire Wire Line
	6400 5950 6400 6100
Wire Wire Line
	6400 6100 6300 6100
$Comp
L power:GND #PWR0130
U 1 1 5E39DC16
P 6500 6100
F 0 "#PWR0130" H 6500 5850 50  0001 C CNN
F 1 "GND" V 6505 5972 50  0000 R CNN
F 2 "" H 6500 6100 50  0001 C CNN
F 3 "" H 6500 6100 50  0001 C CNN
	1    6500 6100
	0    -1   1    0   
$EndComp
Wire Wire Line
	6400 6100 6500 6100
Connection ~ 6400 6100
$Comp
L power:GND #PWR0131
U 1 1 5E39EA2C
P 6000 7300
F 0 "#PWR0131" H 6000 7050 50  0001 C CNN
F 1 "GND" H 6005 7127 50  0000 C CNN
F 2 "" H 6000 7300 50  0001 C CNN
F 3 "" H 6000 7300 50  0001 C CNN
	1    6000 7300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6100 6100 6000 6100
Wire Wire Line
	6000 6100 6000 6200
$Comp
L Device:Battery_Cell BT2
U 1 1 5E3A2444
P 5900 5850
F 0 "BT2" H 5782 5854 50  0000 R CNN
F 1 "Battery_Cell" H 5782 5945 50  0000 R CNN
F 2 "" V 5900 5910 50  0001 C CNN
F 3 "~" V 5900 5910 50  0001 C CNN
	1    5900 5850
	1    0    0    1   
$EndComp
Wire Wire Line
	5900 5750 5900 5500
Wire Wire Line
	5900 5500 6400 5500
Wire Wire Line
	6400 5500 6400 5950
Connection ~ 6400 5950
$Comp
L Device:C_Small C?
U 1 1 5E3A5D9F
P 5800 6100
AR Path="/5E3A5D9F" Ref="C?"  Part="1" 
AR Path="/5E2F29D1/5E3A5D9F" Ref="C23"  Part="1" 
F 0 "C23" V 5700 6000 50  0000 C CNN
F 1 "0.1uF" V 5650 6000 50  0000 C CNN
F 2 "" H 5800 6100 50  0001 C CNN
F 3 "~" H 5800 6100 50  0001 C CNN
	1    5800 6100
	0    1    -1   0   
$EndComp
Wire Wire Line
	5900 6050 5900 6100
Connection ~ 5900 6100
Wire Wire Line
	5900 6100 5900 6200
Wire Wire Line
	5700 6100 5300 6100
Wire Wire Line
	5300 6100 5300 5500
Wire Wire Line
	5300 5500 5900 5500
Connection ~ 5900 5500
Wire Wire Line
	6000 7200 6000 7300
Text Label 5500 6500 2    50   ~ 0
TX_SAM
Text Label 5500 6600 2    50   ~ 0
RX_SAM
NoConn ~ 5500 6800
NoConn ~ 5500 6900
NoConn ~ 6500 6600
NoConn ~ 6500 6700
$Comp
L power:+3.3V #PWR?
U 1 1 5E3B878C
P 6650 6700
F 0 "#PWR?" H 6650 6550 50  0001 C CNN
F 1 "+3.3V" H 6665 6873 50  0000 C CNN
F 2 "" H 6650 6700 50  0001 C CNN
F 3 "" H 6650 6700 50  0001 C CNN
	1    6650 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 6700 6650 6800
Wire Wire Line
	6650 6900 6500 6900
Wire Wire Line
	6500 6800 6650 6800
Connection ~ 6650 6800
Wire Wire Line
	6650 6800 6650 6900
$Comp
L power:+3.3V #PWR?
U 1 1 5E3C0E92
P 7300 2900
F 0 "#PWR?" H 7300 2750 50  0001 C CNN
F 1 "+3.3V" H 7315 3073 50  0000 C CNN
F 2 "" H 7300 2900 50  0001 C CNN
F 3 "" H 7300 2900 50  0001 C CNN
	1    7300 2900
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E3C20EC
P 7300 4700
F 0 "#PWR?" H 7300 4450 50  0001 C CNN
F 1 "GND" H 7305 4527 50  0000 C CNN
F 2 "" H 7300 4700 50  0001 C CNN
F 3 "" H 7300 4700 50  0001 C CNN
	1    7300 4700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7200 4600 7300 4600
Wire Wire Line
	7300 4600 7300 4700
Wire Wire Line
	7300 2900 7300 3000
Wire Wire Line
	7300 3000 7200 3000
Text Label 5000 3500 2    50   ~ 0
XBEE_TX
Text Label 5000 3400 2    50   ~ 0
XBEE_RX
NoConn ~ 7200 4100
NoConn ~ 5000 4600
NoConn ~ 5000 4500
NoConn ~ 5000 4400
NoConn ~ 5000 4300
NoConn ~ 5000 4200
NoConn ~ 5000 4100
NoConn ~ 5000 4000
NoConn ~ 5000 3900
NoConn ~ 5000 3800
NoConn ~ 5000 3700
NoConn ~ 5000 3600
NoConn ~ 5000 3200
NoConn ~ 5000 3100
NoConn ~ 5000 3000
Text Label 5000 1800 2    50   ~ 0
DUST_TX
Text Label 5000 1900 2    50   ~ 0
DUST_RX
Text Notes 5550 2050 0    50   ~ 0
ADD jumper for dust sensor
Text HLabel 1050 1100 0    50   Input ~ 0
SPS_TX
Text HLabel 1050 1200 0    50   Output ~ 0
SPS_RX
Text HLabel 1050 1350 0    50   Input ~ 0
XB_TX
Text HLabel 1050 1450 0    50   Output ~ 0
XB_RX
Text HLabel 1050 1600 0    50   Input ~ 0
GPS_TX
Text HLabel 1050 1700 0    50   Output ~ 0
GPS_RX
Text Label 1050 1100 0    50   ~ 0
DUST_TX
Text Label 1050 1200 0    50   ~ 0
DUST_RX
Text Label 1050 1350 0    50   ~ 0
XBEE_TX
Text Label 1050 1450 0    50   ~ 0
XBEE_RX
Text Label 1050 1600 0    50   ~ 0
TX_SAM
Text Label 1050 1700 0    50   ~ 0
RX_SAM
$EndSCHEMATC