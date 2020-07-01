EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR02
U 1 1 5ABD2774
P 6750 7550
F 0 "#PWR02" H 6750 7300 50  0001 C CNN
F 1 "GND" H 6750 7400 50  0000 C CNN
F 2 "" H 6750 7550 50  0001 C CNN
F 3 "" H 6750 7550 50  0001 C CNN
	1    6750 7550
	1    0    0    -1  
$EndComp
$Comp
L bluepill_ethernet-rescue:Conn_01x03-conn J2
U 1 1 5ABD688D
P 9600 6200
F 0 "J2" H 9600 6400 50  0000 C CNN
F 1 "Conn_01x03" H 9600 6000 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-03_03x2.54mm_Straight" H 9600 6200 50  0001 C CNN
F 3 "" H 9600 6200 50  0001 C CNN
	1    9600 6200
	0    1    1    0   
$EndComp
$Comp
L bluepill_ethernet-rescue:CP-device C4
U 1 1 5ABDC0B1
P 10150 5600
F 0 "C4" H 10175 5700 50  0000 L CNN
F 1 "CP" H 10175 5500 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 10188 5450 50  0001 C CNN
F 3 "" H 10150 5600 50  0001 C CNN
	1    10150 5600
	1    0    0    -1  
$EndComp
$Comp
L bluepill_ethernet-rescue:CP-device C6
U 1 1 5ABEBD7D
P 9900 2750
F 0 "C6" H 9782 2704 50  0000 R CNN
F 1 "CP" H 9782 2795 50  0000 R CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 9938 2600 50  0001 C CNN
F 3 "" H 9900 2750 50  0001 C CNN
	1    9900 2750
	-1   0    0    1   
$EndComp
$Comp
L connectors:HEADER-1x20 J1
U 1 1 5B103221
P 9500 3450
F 0 "J1" H 9628 3486 60  0000 L CNB
F 1 "HEADER-1x20" H 9628 3395 40  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x20_Pitch2.54mm" H 9500 3450 60  0001 C CNN
F 3 "" H 9500 3450 60  0001 C CNN
F 4 "-" H 9450 4600 40  0001 L BNN "Part"
F 5 "Connector" H 9450 4700 40  0001 L BNN "Family"
	1    9500 3450
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR01
U 1 1 5ABD2688
P 11050 650
F 0 "#PWR01" H 11050 500 50  0001 C CNN
F 1 "VCC" H 11050 800 50  0000 C CNN
F 2 "" H 11050 650 50  0001 C CNN
F 3 "" H 11050 650 50  0001 C CNN
	1    11050 650 
	1    0    0    -1  
$EndComp
$Comp
L WIZ850io:WIZ850io U3
U 1 1 5ABD9273
P 2100 2200
F 0 "U3" H 2100 1800 60  0000 C CNN
F 1 "WIZ850io" H 2100 2600 60  0000 C CNN
F 2 "kicad:WIZ850io" H 2000 2100 60  0001 C CNN
F 3 "" H 2000 2100 60  0001 C CNN
	1    2100 2200
	1    0    0    -1  
$EndComp
NoConn ~ 2600 2250
NoConn ~ 2250 1450
NoConn ~ 2250 1050
Connection ~ 1600 1950
Wire Wire Line
	1600 1950 1600 2050
Wire Wire Line
	9350 650  9350 2300
Connection ~ 9350 650 
Wire Wire Line
	9700 650  9700 5450
Connection ~ 9700 650 
Connection ~ 2600 3500
Connection ~ 9700 5450
Wire Wire Line
	10150 5800 10150 5750
Wire Wire Line
	9600 5800 9600 6000
Wire Wire Line
	9350 650  9700 650 
Connection ~ 9900 2600
Wire Wire Line
	9900 2300 9350 2300
Connection ~ 9350 2300
Wire Wire Line
	9350 2300 9350 2500
Wire Wire Line
	1750 1150 700  1150
Wire Wire Line
	700  1150 700  3500
Wire Wire Line
	700  3500 2600 3500
Wire Wire Line
	1750 1250 800  1250
Wire Wire Line
	800  1250 800  3600
Wire Wire Line
	1750 1350 900  1350
Wire Wire Line
	1750 1450 1000 1450
Wire Wire Line
	1000 1450 1000 3400
Wire Wire Line
	2250 1250 3400 1250
Wire Wire Line
	3400 1250 3400 2350
Wire Wire Line
	2250 1350 3300 1350
Wire Wire Line
	900  1350 900  3800
Wire Wire Line
	5200 6650 5200 6750
Wire Wire Line
	4100 6750 3950 6750
Wire Wire Line
	5200 6750 5450 6750
Connection ~ 5450 6750
Wire Wire Line
	1750 1050 700  1050
Wire Wire Line
	700  1050 700  650 
Wire Wire Line
	6750 7000 6750 7550
Wire Wire Line
	2600 2350 3400 2350
Wire Wire Line
	2600 2450 2600 3500
Wire Wire Line
	1600 1600 1600 1950
Wire Wire Line
	3500 1150 3500 1600
Connection ~ 3500 1150
Wire Wire Line
	2250 1150 3500 1150
Connection ~ 3500 1600
Wire Wire Line
	3500 1600 1600 1600
$Comp
L bluepill_ethernet-rescue:C-device C1
U 1 1 5ABDB1C7
P 3650 900
F 0 "C1" H 3675 1000 50  0000 L CNN
F 1 "C" H 3675 800 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 3688 750 50  0001 C CNN
F 3 "" H 3650 900 50  0001 C CNN
	1    3650 900 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3500 900  3500 1150
Wire Wire Line
	4000 650  4000 900 
Connection ~ 4000 650 
Connection ~ 4000 900 
Wire Wire Line
	700  650  4000 650 
Wire Wire Line
	3800 900  4000 900 
Wire Wire Line
	4000 2050 4000 2150
Connection ~ 4000 2050
Wire Wire Line
	2600 2050 4000 2050
Wire Wire Line
	4000 900  4000 2050
Wire Wire Line
	4000 2150 2600 2150
Connection ~ 3500 1950
Wire Wire Line
	2600 1950 3500 1950
Wire Wire Line
	3500 1950 3500 1600
Wire Wire Line
	1500 3800 1500 2350
Connection ~ 1500 3800
Wire Wire Line
	900  3800 1500 3800
Wire Wire Line
	1500 2350 1600 2350
Wire Wire Line
	1400 3400 1400 2250
Wire Wire Line
	1000 3400 1400 3400
Connection ~ 1400 3400
Wire Wire Line
	1400 2250 1600 2250
Wire Wire Line
	800  3600 1300 3600
Connection ~ 3400 2350
Wire Wire Line
	1600 2150 1300 2150
Wire Wire Line
	1300 2150 1300 3600
Wire Wire Line
	3400 2350 3400 3300
Connection ~ 1300 3600
Wire Wire Line
	4100 6650 3850 6650
Wire Wire Line
	4100 6550 3750 6550
$Comp
L bluepill_ethernet-rescue:C-device C5
U 1 1 5ABDB608
P 5450 6600
F 0 "C5" H 5475 6700 50  0000 L CNN
F 1 "C" H 5475 6500 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 5488 6450 50  0001 C CNN
F 3 "" H 5450 6600 50  0001 C CNN
	1    5450 6600
	-1   0    0    1   
$EndComp
Wire Wire Line
	5200 6450 5450 6450
Wire Wire Line
	5450 6750 5450 7000
Wire Wire Line
	3500 7000 5450 7000
Wire Wire Line
	5450 7000 5700 7000
Connection ~ 5450 7000
Wire Wire Line
	6750 7000 6750 6450
Wire Wire Line
	6750 6450 7600 6450
Wire Wire Line
	11050 6450 11050 5800
Connection ~ 6750 7000
Wire Wire Line
	5450 650  5450 5800
Connection ~ 5450 650 
Connection ~ 7600 6450
Wire Wire Line
	7600 6450 11050 6450
Wire Wire Line
	11050 5800 11050 2600
$Comp
L bluepill_ethernet-rescue:CP-device C2
U 1 1 5ABDC134
P 11050 900
F 0 "C2" H 11075 1000 50  0000 L CNN
F 1 "CP" H 11075 800 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 11088 750 50  0001 C CNN
F 3 "" H 11050 900 50  0001 C CNN
	1    11050 900 
	1    0    0    -1  
$EndComp
Connection ~ 11050 5800
Wire Wire Line
	11050 750  11050 650 
Connection ~ 11050 650 
Connection ~ 10150 5800
Wire Wire Line
	10150 5800 11050 5800
Wire Wire Line
	9900 2600 11050 2600
Connection ~ 11050 2600
Wire Wire Line
	11050 2600 11050 1050
Wire Wire Line
	9350 2600 9900 2600
Wire Wire Line
	7600 4300 7600 4400
Connection ~ 7600 4400
Wire Wire Line
	3650 6450 4100 6450
Wire Wire Line
	3650 3700 3650 5800
Connection ~ 5450 5800
Wire Wire Line
	5450 5800 5450 6450
Wire Wire Line
	5700 6100 5700 7000
Connection ~ 5700 7000
Wire Wire Line
	3950 6100 3950 6750
Wire Wire Line
	3750 5900 3750 6550
Connection ~ 3650 5800
Wire Wire Line
	3650 5800 3650 6450
Connection ~ 5450 6450
Connection ~ 5200 6750
$Comp
L 93c46:93C46 U2
U 1 1 5ABD4EFE
P 4650 6600
F 0 "U2" H 4650 6850 60  0000 C CNN
F 1 "93C46" H 4650 6325 60  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 4650 6600 60  0001 C CNN
F 3 "" H 4650 6600 60  0001 C CNN
	1    4650 6600
	1    0    0    -1  
$EndComp
NoConn ~ 5200 6550
Wire Wire Line
	9700 5450 10150 5450
Wire Wire Line
	9600 5800 10150 5800
Wire Wire Line
	3850 6000 3850 6650
Wire Wire Line
	5200 5800 5450 5800
NoConn ~ 5200 5900
Connection ~ 5200 6100
Wire Wire Line
	5200 6100 5700 6100
Wire Wire Line
	5200 6000 5200 6100
Wire Wire Line
	4100 6100 3950 6100
Wire Wire Line
	4100 6000 3850 6000
Wire Wire Line
	4100 5900 3750 5900
Wire Wire Line
	4100 5800 3650 5800
$Comp
L 93c46:93C46 U4
U 1 1 5F427178
P 4650 5950
F 0 "U4" H 4650 6200 60  0000 C CNN
F 1 "93C46" H 4650 5675 60  0000 C CNN
F 2 "Package_SO:SOIC-8_5.23x5.23mm_P1.27mm" H 4650 5950 60  0001 C CNN
F 3 "" H 4650 5950 60  0001 C CNN
	1    4650 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 2900 9800 2700
Wire Wire Line
	9900 2900 9800 2900
Wire Wire Line
	9800 2700 9350 2700
$Comp
L bluepill_ethernet-rescue:C-device C7
U 1 1 5ABF15CD
P 9900 2450
F 0 "C7" H 9925 2550 50  0000 L CNN
F 1 "C" H 9925 2350 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 9938 2300 50  0001 C CNN
F 3 "" H 9900 2450 50  0001 C CNN
	1    9900 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 4400 7600 6450
Wire Wire Line
	5700 7000 6750 7000
$Comp
L bluepill_ethernet-rescue:SW_Push-switches SW1
U 1 1 5ADFA2DD
P 5600 2500
F 0 "SW1" V 5554 2648 50  0000 L CNN
F 1 "SW_Push" V 5645 2648 50  0000 L CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 5600 2700 50  0001 C CNN
F 3 "" H 5600 2700 50  0001 C CNN
	1    5600 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 2300 5600 650 
Connection ~ 5600 650 
Wire Wire Line
	5600 650  5450 650 
Wire Wire Line
	5600 2900 5600 2700
Connection ~ 6150 650 
Wire Wire Line
	6150 650  5600 650 
$Comp
L connectors:HEADER-1x20 J4
U 1 1 5F425C06
P 6750 3450
F 0 "J4" H 6878 3486 60  0000 L CNB
F 1 "HEADER-1x20" H 6878 3395 40  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x20_Pitch2.54mm" H 6750 3450 60  0001 C CNN
F 3 "" H 6750 3450 60  0001 C CNN
F 4 "-" H 6700 4600 40  0001 L BNN "Part"
F 5 "Connector" H 6700 4700 40  0001 L BNN "Family"
	1    6750 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2900 6600 2900
Wire Wire Line
	6600 2900 5600 2900
Connection ~ 6600 2900
Wire Wire Line
	7600 3000 6600 3000
Connection ~ 6600 3000
Wire Wire Line
	7600 3100 6600 3100
Wire Wire Line
	7600 3200 6600 3200
Wire Wire Line
	6600 3200 5800 3200
Connection ~ 6600 3200
Wire Wire Line
	3400 3300 6600 3300
Wire Wire Line
	6600 3300 7600 3300
Connection ~ 6600 3300
Wire Wire Line
	6600 3400 7600 3400
Connection ~ 6600 3400
Wire Wire Line
	6600 3500 7600 3500
Connection ~ 6600 3500
Wire Wire Line
	6600 3600 7600 3600
Connection ~ 6600 3600
Wire Wire Line
	3650 3700 6600 3700
Wire Wire Line
	6600 3700 7600 3700
Connection ~ 6600 3700
Wire Wire Line
	7600 3800 6600 3800
Wire Wire Line
	6600 3800 1500 3800
Connection ~ 6600 3800
Wire Wire Line
	6600 3900 7600 3900
Wire Wire Line
	7600 4200 6600 4200
Wire Wire Line
	6600 4200 6150 4200
Connection ~ 6600 4200
Wire Wire Line
	6600 2600 7600 2600
Wire Wire Line
	7600 4000 6600 4000
Wire Wire Line
	6600 4100 7600 4100
Wire Wire Line
	7600 4300 6600 4300
Wire Wire Line
	6600 4400 7600 4400
Wire Wire Line
	5800 3200 5800 5350
Wire Wire Line
	5800 5350 9500 5350
Wire Wire Line
	3300 1350 3300 3000
Connection ~ 3300 3000
Wire Wire Line
	3300 3000 6600 3000
Wire Wire Line
	1600 2450 1600 3000
Wire Wire Line
	1600 3000 3300 3000
Connection ~ 6600 3900
Connection ~ 6600 4000
Wire Wire Line
	6600 3900 4550 3900
Wire Wire Line
	4550 3900 600  3900
Connection ~ 4550 3900
Wire Wire Line
	700  4000 4650 4000
Wire Wire Line
	4750 4000 6600 4000
Connection ~ 4750 4000
Wire Wire Line
	3750 5900 3750 3400
Connection ~ 3750 5900
Wire Wire Line
	3750 3400 6600 3400
Connection ~ 3750 3400
Wire Wire Line
	3850 6000 3850 3600
Connection ~ 3850 6000
Wire Wire Line
	3850 3600 6600 3600
Connection ~ 3850 3600
Wire Wire Line
	3950 6100 3950 3500
Connection ~ 3950 6100
Wire Wire Line
	2600 3500 3950 3500
Wire Wire Line
	3950 3500 6600 3500
Connection ~ 3950 3500
Wire Wire Line
	1300 3600 3850 3600
Wire Wire Line
	1400 3400 3750 3400
Wire Wire Line
	1800 5150 1950 4550
Wire Wire Line
	1950 4850 1800 5250
Wire Wire Line
	1800 5350 1950 5150
Wire Wire Line
	1950 5450 1800 5450
Wire Wire Line
	1800 5550 1950 5750
Wire Wire Line
	1950 6050 1800 5650
Wire Wire Line
	1800 5750 1950 6350
Wire Wire Line
	1950 6650 1800 5850
Wire Wire Line
	3500 7000 2900 7000
Connection ~ 3500 7000
$Comp
L bluepill_ethernet-rescue:Conn_02x05_Odd_Even-conn J3
U 1 1 5B1F5D26
P 1950 1250
F 0 "J3" H 2000 1667 50  0000 C CNN
F 1 "Conn_02x05_Odd_Even" H 2000 1576 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x05_Pitch2.54mm" H 1950 1250 50  0001 C CNN
F 3 "~" H 1950 1250 50  0001 C CNN
	1    1950 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 2700 6600 2700
Wire Wire Line
	6600 2800 6950 2800
Wire Wire Line
	7600 2700 7450 2700
Wire Wire Line
	7600 2800 7450 2800
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J5
U 1 1 5F1F9341
P 7150 2700
F 0 "J5" H 7200 2917 50  0000 C CNN
F 1 "Conn_02x02_Odd_Even" H 7200 2826 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 7150 2700 50  0001 C CNN
F 3 "~" H 7150 2700 50  0001 C CNN
	1    7150 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6800 3500 7000
Wire Wire Line
	3500 7050 3500 7000
Connection ~ 6600 2700
Wire Wire Line
	6600 2700 6400 2700
Wire Wire Line
	6300 2800 6600 2800
Connection ~ 6600 2800
Text GLabel 5000 2400 3    60   Input ~ 0
dled1
Text GLabel 5000 1300 1    60   Input ~ 0
dled2
$Comp
L Device:R_Network08 RN2
U 1 1 5F2A9077
P 10450 850
F 0 "RN2" H 9970 896 50  0000 R CNN
F 1 "4,7k" H 9970 805 50  0000 R CNN
F 2 "Resistor_THT:R_Array_SIP9" V 10925 850 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 10450 850 50  0001 C CNN
	1    10450 850 
	-1   0    0    -1  
$EndComp
Text GLabel 10550 1350 3    60   Input ~ 0
dled1
Text GLabel 10450 1350 3    60   Input ~ 0
dled2
Wire Wire Line
	11050 650  10850 650 
Wire Wire Line
	10850 650  9700 650 
Connection ~ 10850 650 
Wire Wire Line
	9500 5350 10850 5350
Wire Wire Line
	10850 5350 10850 1950
Connection ~ 9500 5350
$Comp
L Device:LED D1
U 1 1 5F306567
P 5000 2250
F 0 "D1" V 4947 2328 50  0000 L CNN
F 1 "LED" V 5038 2328 50  0000 L CNN
F 2 "LED_THT:LED_D3.0mm" H 5000 2250 50  0001 C CNN
F 3 "~" H 5000 2250 50  0001 C CNN
	1    5000 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 650  6150 4200
$Comp
L Device:LED D2
U 1 1 5F305A99
P 5000 1450
F 0 "D2" V 5039 1333 50  0000 R CNN
F 1 "LED" V 4948 1333 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm" H 5000 1450 50  0001 C CNN
F 3 "~" H 5000 1450 50  0001 C CNN
	1    5000 1450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9500 5350 9500 6000
Wire Wire Line
	9700 5450 9700 6000
Text GLabel 10250 1350 3    60   Input ~ 0
i2c1c
Text GLabel 10150 1350 3    60   Input ~ 0
i2c1d
Wire Wire Line
	4000 650  4800 650 
Wire Wire Line
	4750 4000 4750 4150
Wire Wire Line
	4550 4150 4550 3900
Wire Wire Line
	6400 1600 5100 1600
Wire Wire Line
	6400 1600 6400 2700
Wire Wire Line
	6300 2100 5100 2100
Wire Wire Line
	6300 2100 6300 2800
Wire Wire Line
	4550 3900 4550 2200
Connection ~ 4650 4000
Wire Wire Line
	4650 4000 4750 4000
Text GLabel 4750 4150 2    60   Input ~ 0
i2c1d
Text GLabel 4550 4150 0    60   Input ~ 0
i2c1c
Wire Wire Line
	10750 3000 10750 1350
Wire Wire Line
	9350 3000 10750 3000
Wire Wire Line
	9350 3100 10650 3100
$Comp
L power:VCC #PWR0101
U 1 1 5F05A26E
P 1300 4400
F 0 "#PWR0101" H 1300 4250 50  0001 C CNN
F 1 "VCC" H 1300 4550 50  0000 C CNN
F 2 "" H 1300 4400 50  0001 C CNN
F 3 "" H 1300 4400 50  0001 C CNN
	1    1300 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4550 2500 5150
$Comp
L Device:LED D4
U 1 1 5F09219D
P 2100 4850
F 0 "D4" H 2093 5066 50  0000 C CNN
F 1 "LED" H 2093 4975 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 2100 4850 50  0001 C CNN
F 3 "~" H 2100 4850 50  0001 C CNN
	1    2100 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 5250 2250 4850
Wire Wire Line
	2250 5150 2500 5350
Wire Wire Line
	2500 5450 2250 5450
Wire Wire Line
	2250 5750 2500 5550
Wire Wire Line
	2500 5650 2250 6050
Wire Wire Line
	2250 6350 2500 5750
Wire Wire Line
	2500 5850 2250 6650
Wire Wire Line
	600  3900 600  5150
Wire Wire Line
	700  4000 700  5250
Wire Wire Line
	600  5150 800  5150
Wire Wire Line
	700  5250 800  5250
Wire Wire Line
	1300 7000 1300 6250
$Comp
L Interface_Expansion:PCF8574A U5
U 1 1 5F1937B2
P 1300 5550
F 0 "U5" H 1300 6431 50  0000 C CNN
F 1 "PCF8574A" H 1300 6340 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm" H 1300 5550 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/PCF8574_PCF8574A.pdf" H 1300 5550 50  0001 C CNN
	1    1300 5550
	1    0    0    -1  
$EndComp
NoConn ~ 800  5950
Wire Wire Line
	800  5450 650  5450
Wire Wire Line
	650  5450 650  5550
Wire Wire Line
	650  5550 800  5550
Wire Wire Line
	800  5650 650  5650
Wire Wire Line
	650  5650 650  5550
Connection ~ 650  5550
Wire Wire Line
	650  5650 650  7000
Wire Wire Line
	650  7000 1300 7000
Connection ~ 650  5650
Connection ~ 1300 7000
Wire Wire Line
	3500 1950 3500 7000
Wire Wire Line
	1300 4850 1300 4500
Wire Wire Line
	1300 4500 2900 4500
Wire Wire Line
	2900 4500 2900 5150
Connection ~ 1300 4500
Wire Wire Line
	1300 4500 1300 4400
$Comp
L Device:LED D3
U 1 1 5F74D6FC
P 2100 4550
F 0 "D3" H 2093 4766 50  0000 C CNN
F 1 "LED" H 2093 4675 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 2100 4550 50  0001 C CNN
F 3 "~" H 2100 4550 50  0001 C CNN
	1    2100 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D5
U 1 1 5F74D8AD
P 2100 5150
F 0 "D5" H 2093 5366 50  0000 C CNN
F 1 "LED" H 2093 5275 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 2100 5150 50  0001 C CNN
F 3 "~" H 2100 5150 50  0001 C CNN
	1    2100 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D6
U 1 1 5F74DE69
P 2100 5450
F 0 "D6" H 2093 5666 50  0000 C CNN
F 1 "LED" H 2093 5575 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 2100 5450 50  0001 C CNN
F 3 "~" H 2100 5450 50  0001 C CNN
	1    2100 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D7
U 1 1 5F74E0C9
P 2100 5750
F 0 "D7" H 2093 5966 50  0000 C CNN
F 1 "LED" H 2093 5875 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 2100 5750 50  0001 C CNN
F 3 "~" H 2100 5750 50  0001 C CNN
	1    2100 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D8
U 1 1 5F74E2FE
P 2100 6050
F 0 "D8" H 2093 6266 50  0000 C CNN
F 1 "LED" H 2093 6175 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 2100 6050 50  0001 C CNN
F 3 "~" H 2100 6050 50  0001 C CNN
	1    2100 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D9
U 1 1 5F74E584
P 2100 6350
F 0 "D9" H 2093 6566 50  0000 C CNN
F 1 "LED" H 2093 6475 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 2100 6350 50  0001 C CNN
F 3 "~" H 2100 6350 50  0001 C CNN
	1    2100 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D10
U 1 1 5F74E791
P 2100 6650
F 0 "D10" H 2093 6866 50  0000 C CNN
F 1 "LED" H 2093 6775 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 2100 6650 50  0001 C CNN
F 3 "~" H 2100 6650 50  0001 C CNN
	1    2100 6650
	1    0    0    -1  
$EndComp
Connection ~ 6600 4100
Wire Wire Line
	6600 4100 5700 4100
Connection ~ 6600 3100
Wire Wire Line
	6600 3100 5700 3100
Wire Wire Line
	5700 3100 5700 4100
Wire Wire Line
	10350 1950 10850 1950
$Comp
L Jumper:SolderJumper_2_Bridged JP1
U 1 1 5F765C2C
P 10350 1800
F 0 "JP1" V 10304 1868 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 10395 1868 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 10350 1800 50  0001 C CNN
F 3 "~" H 10350 1800 50  0001 C CNN
	1    10350 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	10850 1650 10850 1050
Wire Wire Line
	10350 1650 10350 1050
$Comp
L Jumper:SolderJumper_2_Bridged JP2
U 1 1 5F782A60
P 10850 1800
F 0 "JP2" V 10804 1868 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 10895 1868 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 10850 1800 50  0001 C CNN
F 3 "~" H 10850 1800 50  0001 C CNN
	1    10850 1800
	0    1    1    0   
$EndComp
Connection ~ 10850 1950
$Comp
L Device:R R1
U 1 1 5F8367EB
P 2650 5150
F 0 "R1" V 2443 5150 50  0000 C CNN
F 1 "R" V 2534 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 5150 50  0001 C CNN
F 3 "~" H 2650 5150 50  0001 C CNN
	1    2650 5150
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5F83AB09
P 2650 5250
F 0 "R2" V 2443 5250 50  0000 C CNN
F 1 "R" V 2534 5250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 5250 50  0001 C CNN
F 3 "~" H 2650 5250 50  0001 C CNN
	1    2650 5250
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5F83AD2B
P 2650 5350
F 0 "R3" V 2443 5350 50  0000 C CNN
F 1 "R" V 2534 5350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 5350 50  0001 C CNN
F 3 "~" H 2650 5350 50  0001 C CNN
	1    2650 5350
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5F83AF8D
P 2650 5450
F 0 "R4" V 2443 5450 50  0000 C CNN
F 1 "R" V 2534 5450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 5450 50  0001 C CNN
F 3 "~" H 2650 5450 50  0001 C CNN
	1    2650 5450
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5F83B1A6
P 2650 5550
F 0 "R5" V 2443 5550 50  0000 C CNN
F 1 "R" V 2534 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 5550 50  0001 C CNN
F 3 "~" H 2650 5550 50  0001 C CNN
	1    2650 5550
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5F83B3B4
P 2650 5650
F 0 "R6" V 2443 5650 50  0000 C CNN
F 1 "R" V 2534 5650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 5650 50  0001 C CNN
F 3 "~" H 2650 5650 50  0001 C CNN
	1    2650 5650
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5F83B558
P 2650 5750
F 0 "R7" V 2443 5750 50  0000 C CNN
F 1 "R" V 2534 5750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 5750 50  0001 C CNN
F 3 "~" H 2650 5750 50  0001 C CNN
	1    2650 5750
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5F83B73C
P 2650 5850
F 0 "R8" V 2443 5850 50  0000 C CNN
F 1 "R" V 2534 5850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 5850 50  0001 C CNN
F 3 "~" H 2650 5850 50  0001 C CNN
	1    2650 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	2800 5850 2900 5850
Wire Wire Line
	2800 5750 2900 5750
Connection ~ 2900 5750
Wire Wire Line
	2900 5750 2900 5850
Wire Wire Line
	2900 5650 2800 5650
Connection ~ 2900 5650
Wire Wire Line
	2900 5650 2900 5750
Wire Wire Line
	2800 5550 2900 5550
Connection ~ 2900 5550
Wire Wire Line
	2900 5550 2900 5650
Wire Wire Line
	2900 5450 2800 5450
Connection ~ 2900 5450
Wire Wire Line
	2900 5450 2900 5550
Wire Wire Line
	2800 5350 2900 5350
Connection ~ 2900 5350
Wire Wire Line
	2900 5350 2900 5450
Wire Wire Line
	2900 5250 2800 5250
Connection ~ 2900 5250
Wire Wire Line
	2900 5250 2900 5350
Wire Wire Line
	2800 5150 2900 5150
Connection ~ 2900 5150
Wire Wire Line
	2900 5150 2900 5250
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J7
U 1 1 5F9EE42C
P 4900 1900
F 0 "J7" V 4996 1612 50  0000 R CNN
F 1 "Conn_02x04_Odd_Even" V 4905 1612 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 4900 1900 50  0001 C CNN
F 3 "~" H 4900 1900 50  0001 C CNN
	1    4900 1900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4550 2200 4900 2200
Wire Wire Line
	4900 2200 4900 2100
Wire Wire Line
	4900 1600 4900 1450
Wire Wire Line
	4900 1450 4650 1450
Wire Wire Line
	4650 1450 4650 4000
Wire Wire Line
	4800 1600 4800 650 
Connection ~ 4800 650 
Wire Wire Line
	4800 650  5450 650 
$Comp
L power:GND #PWR0102
U 1 1 5FA1CA23
P 4800 2350
F 0 "#PWR0102" H 4800 2100 50  0001 C CNN
F 1 "GND" H 4800 2200 50  0000 C CNN
F 2 "" H 4800 2350 50  0001 C CNN
F 3 "" H 4800 2350 50  0001 C CNN
	1    4800 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 2350 4800 2100
$Comp
L Sensor_Temperature:LM75B U6
U 1 1 5FA9C565
P 2600 9750
F 0 "U6" H 2600 10431 50  0000 C CNN
F 1 "LM75B" H 2600 10340 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2600 9750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm75b.pdf" H 2600 9750 50  0001 C CNN
	1    2600 9750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 9650 700  9650
Wire Wire Line
	700  9650 700  5250
Connection ~ 700  5250
Wire Wire Line
	600  5150 600  9750
Wire Wire Line
	600  9750 2200 9750
Connection ~ 600  5150
$Comp
L power:GND #PWR0103
U 1 1 5FABCE5C
P 2600 10250
F 0 "#PWR0103" H 2600 10000 50  0001 C CNN
F 1 "GND" H 2600 10100 50  0000 C CNN
F 2 "" H 2600 10250 50  0001 C CNN
F 3 "" H 2600 10250 50  0001 C CNN
	1    2600 10250
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0104
U 1 1 5FABDC40
P 2600 9250
F 0 "#PWR0104" H 2600 9100 50  0001 C CNN
F 1 "VCC" H 2600 9400 50  0000 C CNN
F 2 "" H 2600 9250 50  0001 C CNN
F 3 "" H 2600 9250 50  0001 C CNN
	1    2600 9250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 9650 3000 9750
Wire Wire Line
	3000 9850 3000 9750
Connection ~ 3000 9750
Wire Wire Line
	3000 9850 3000 10250
Wire Wire Line
	3000 10250 2600 10250
Connection ~ 3000 9850
Connection ~ 2600 10250
Connection ~ 9350 2500
Connection ~ 9350 2600
Connection ~ 9350 2700
Connection ~ 9350 3000
Connection ~ 9350 3100
Connection ~ 7600 4300
$Comp
L bluepill_breakouts:BluePill_STM32F103C U1
U 1 1 5ABD20EB
P 8450 3200
F 0 "U1" H 8000 1850 50  0000 C CNN
F 1 "BluePill_STM32F103C" H 8500 4050 50  0000 C CNN
F 2 "BluePill_breakouts:BluePill_STM32F103C" H 8500 1600 50  0001 C CNN
F 3 "www.rogerclark.net" H 8450 1700 50  0001 C CNN
	1    8450 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2500 7600 2500
Wire Wire Line
	6150 650  9350 650 
$Comp
L Jumper:SolderJumper_2_Bridged JP3
U 1 1 5FB7A3F5
P 10150 1200
F 0 "JP3" V 10104 1268 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 10195 1268 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 10150 1200 50  0001 C CNN
F 3 "~" H 10150 1200 50  0001 C CNN
	1    10150 1200
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Bridged JP4
U 1 1 5FB7AA5A
P 10250 1200
F 0 "JP4" V 10204 1268 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 10295 1268 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 10250 1200 50  0001 C CNN
F 3 "~" H 10250 1200 50  0001 C CNN
	1    10250 1200
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Bridged JP5
U 1 1 5FB7B04B
P 10450 1200
F 0 "JP5" V 10404 1268 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 10495 1268 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 10450 1200 50  0001 C CNN
F 3 "~" H 10450 1200 50  0001 C CNN
	1    10450 1200
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Bridged JP6
U 1 1 5FB7B548
P 10550 1200
F 0 "JP6" V 10504 1268 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 10595 1268 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 10550 1200 50  0001 C CNN
F 3 "~" H 10550 1200 50  0001 C CNN
	1    10550 1200
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Bridged JP7
U 1 1 5FB7C981
P 10650 1200
F 0 "JP7" V 10604 1268 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 10695 1268 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 10650 1200 50  0001 C CNN
F 3 "~" H 10650 1200 50  0001 C CNN
	1    10650 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	10650 1350 10650 3100
$Comp
L Jumper:SolderJumper_2_Bridged JP8
U 1 1 5FB7D65E
P 10750 1200
F 0 "JP8" V 10704 1268 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 10795 1268 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm" H 10750 1200 50  0001 C CNN
F 3 "~" H 10750 1200 50  0001 C CNN
	1    10750 1200
	0    1    1    0   
$EndComp
$Comp
L bluepill_ethernet-rescue:C-device C3
U 1 1 5FBB6638
P 2900 6400
F 0 "C3" H 2925 6500 50  0000 L CNN
F 1 "C" H 2925 6300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2938 6250 50  0001 C CNN
F 3 "" H 2900 6400 50  0001 C CNN
	1    2900 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 6250 2900 5850
Connection ~ 2900 5850
Wire Wire Line
	2900 6550 2900 7000
Connection ~ 2900 7000
Wire Wire Line
	2900 7000 1300 7000
$EndSCHEMATC
