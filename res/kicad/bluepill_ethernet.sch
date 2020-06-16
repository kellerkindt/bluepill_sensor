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
Connection ~ 9350 2500
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
F 2 "Housings_SSOP:TSSOP-8_4.4x3mm_Pitch0.65mm" H 4650 5950 60  0001 C CNN
F 3 "" H 4650 5950 60  0001 C CNN
	1    4650 5950
	1    0    0    -1  
$EndComp
Connection ~ 9350 3100
Connection ~ 9350 3000
Connection ~ 9350 2700
Connection ~ 9350 2600
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
	9700 5450 9700 5750
Wire Wire Line
	9700 5750 9700 6000
Connection ~ 9700 5750
Wire Wire Line
	9700 5750 9300 5750
Wire Wire Line
	9500 5450 9500 6000
Wire Wire Line
	9500 5350 9500 5450
Connection ~ 9500 5450
Wire Wire Line
	9300 5450 9500 5450
$Comp
L bluepill_ethernet-rescue:R-device R1
U 1 1 5ABD825B
P 9300 5600
F 0 "R1" V 9380 5600 50  0000 C CNN
F 1 "R" V 9300 5600 50  0000 C CNN
F 2 "Resistors_Universal:Resistor_SMD+THTuniversal_0805to1206_RM10_HandSoldering" V 9230 5600 50  0001 C CNN
F 3 "" H 9300 5600 50  0001 C CNN
	1    9300 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 3000 10300 3000
Wire Wire Line
	10300 3000 10300 2800
Wire Wire Line
	10450 3100 9350 3100
Wire Wire Line
	10450 2800 10450 3100
$Comp
L bluepill_ethernet-rescue:R-device R2
U 1 1 5B105A17
P 10450 2650
F 0 "R2" V 10530 2650 50  0000 C CNN
F 1 "R" V 10450 2650 50  0000 C CNN
F 2 "Resistors_Universal:Resistor_SMD+THTuniversal_0805to1206_RM10_HandSoldering" V 10380 2650 50  0001 C CNN
F 3 "" H 10450 2650 50  0001 C CNN
	1    10450 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 2500 10300 2500
Wire Wire Line
	10450 2500 10300 2500
Connection ~ 10300 2500
$Comp
L bluepill_ethernet-rescue:R-device R3
U 1 1 5B105160
P 10300 2650
F 0 "R3" V 10380 2650 50  0000 C CNN
F 1 "R" V 10300 2650 50  0000 C CNN
F 2 "Resistors_Universal:Resistor_SMD+THTuniversal_0805to1206_RM10_HandSoldering" V 10230 2650 50  0001 C CNN
F 3 "" H 10300 2650 50  0001 C CNN
	1    10300 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	11050 650  9700 650 
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
	3500 7000 3500 1950
Connection ~ 3750 6550
Wire Wire Line
	3750 6550 3000 6550
Connection ~ 3850 6650
Wire Wire Line
	3850 6650 2800 6650
Connection ~ 3950 6750
Wire Wire Line
	3950 6750 2600 6750
Wire Wire Line
	1400 3400 3000 3400
Connection ~ 3000 3400
Wire Wire Line
	3000 3400 3000 6550
Wire Wire Line
	1300 3600 2800 3600
Connection ~ 2800 3600
Wire Wire Line
	2800 3600 2800 6650
Wire Wire Line
	2600 3500 2600 6750
Wire Wire Line
	5450 650  4000 650 
Wire Wire Line
	1600 2450 1600 3900
Connection ~ 3300 3900
Wire Wire Line
	1600 3900 3300 3900
Wire Wire Line
	3300 1350 3300 3900
Wire Wire Line
	7600 4100 7600 4000
Wire Wire Line
	3300 3900 7600 3900
Wire Wire Line
	7600 3800 1500 3800
Wire Wire Line
	3650 3700 7600 3700
Wire Wire Line
	2800 3600 7600 3600
Wire Wire Line
	2600 3500 7600 3500
Wire Wire Line
	3000 3400 7600 3400
Wire Wire Line
	3400 3300 7600 3300
Wire Wire Line
	7350 5350 9500 5350
Wire Wire Line
	7600 2800 7350 2800
Wire Wire Line
	7350 2800 7350 5350
Wire Wire Line
	9350 650  7250 650 
Connection ~ 7250 650 
Wire Wire Line
	7600 4200 7250 4200
Wire Wire Line
	7250 4200 7250 650 
Wire Wire Line
	7600 3200 6600 3200
Wire Wire Line
	6600 3200 6600 4150
Wire Wire Line
	7600 3100 6400 3100
Wire Wire Line
	6400 3100 6400 4150
Wire Wire Line
	7600 3000 6150 3000
Wire Wire Line
	6150 3000 6150 4150
Wire Wire Line
	7600 2900 6500 2900
Wire Wire Line
	6500 650  5450 650 
Connection ~ 6500 650 
Wire Wire Line
	7250 650  6500 650 
Wire Wire Line
	6500 2900 6500 2750
Wire Wire Line
	6500 2350 6500 650 
$Comp
L bluepill_ethernet-rescue:SW_Push-switches SW1
U 1 1 5ADFA2DD
P 6500 2550
F 0 "SW1" V 6454 2698 50  0000 L CNN
F 1 "SW_Push" V 6545 2698 50  0000 L CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 6500 2750 50  0001 C CNN
F 3 "" H 6500 2750 50  0001 C CNN
	1    6500 2550
	0    1    1    0   
$EndComp
Connection ~ 7600 4300
Connection ~ 7600 4200
Connection ~ 7600 4100
Connection ~ 7600 4000
Connection ~ 7600 3900
Connection ~ 7600 3800
Connection ~ 7600 3700
Connection ~ 7600 3600
Connection ~ 7600 3500
Connection ~ 7600 3400
Connection ~ 7600 3300
Connection ~ 7600 3200
Connection ~ 7600 3100
Connection ~ 7600 3000
Connection ~ 7600 2900
Connection ~ 7600 2800
$Comp
L connectors:HEADER-1x20 J4
U 1 1 5F425C06
P 7750 3450
F 0 "J4" H 7878 3486 60  0000 L CNB
F 1 "HEADER-1x20" H 7878 3395 40  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x20_Pitch2.54mm" H 7750 3450 60  0001 C CNN
F 3 "" H 7750 3450 60  0001 C CNN
F 4 "-" H 7700 4600 40  0001 L BNN "Part"
F 5 "Connector" H 7700 4700 40  0001 L BNN "Family"
	1    7750 3450
	1    0    0    -1  
$EndComp
$Comp
L bluepill_ethernet-rescue:R-device R6
U 1 1 5B129C71
P 6600 4600
F 0 "R6" V 6680 4600 50  0000 C CNN
F 1 "R" V 6600 4600 50  0000 C CNN
F 2 "Resistors_Universal:Resistor_SMD+THTuniversal_0805to1206_RM10_HandSoldering" V 6530 4600 50  0001 C CNN
F 3 "" H 6600 4600 50  0001 C CNN
	1    6600 4600
	1    0    0    -1  
$EndComp
$Comp
L bluepill_ethernet-rescue:R-device R5
U 1 1 5B129C2D
P 6400 4600
F 0 "R5" V 6480 4600 50  0000 C CNN
F 1 "R" V 6400 4600 50  0000 C CNN
F 2 "Resistors_Universal:Resistor_SMD+THTuniversal_0805to1206_RM10_HandSoldering" V 6330 4600 50  0001 C CNN
F 3 "" H 6400 4600 50  0001 C CNN
	1    6400 4600
	1    0    0    -1  
$EndComp
$Comp
L bluepill_ethernet-rescue:R-device R4
U 1 1 5B129B3F
P 6150 4600
F 0 "R4" V 6230 4600 50  0000 C CNN
F 1 "R" V 6150 4600 50  0000 C CNN
F 2 "Resistors_Universal:Resistor_SMD+THTuniversal_0805to1206_RM10_HandSoldering" V 6080 4600 50  0001 C CNN
F 3 "" H 6150 4600 50  0001 C CNN
	1    6150 4600
	1    0    0    -1  
$EndComp
$Comp
L bluepill_ethernet-rescue:LED-device D3
U 1 1 5B11DE60
P 6600 4300
F 0 "D3" V 6638 4183 50  0000 R CNN
F 1 "LED" V 6547 4183 50  0000 R CNN
F 2 "LEDs:LED_D3.0mm" H 6600 4300 50  0001 C CNN
F 3 "" H 6600 4300 50  0001 C CNN
	1    6600 4300
	0    -1   -1   0   
$EndComp
$Comp
L bluepill_ethernet-rescue:LED-device D2
U 1 1 5B11DDF6
P 6400 4300
F 0 "D2" V 6438 4183 50  0000 R CNN
F 1 "LED" V 6347 4183 50  0000 R CNN
F 2 "LEDs:LED_D3.0mm" H 6400 4300 50  0001 C CNN
F 3 "" H 6400 4300 50  0001 C CNN
	1    6400 4300
	0    -1   -1   0   
$EndComp
$Comp
L bluepill_ethernet-rescue:LED-device D1
U 1 1 5B11D9D9
P 6150 4300
F 0 "D1" V 6188 4183 50  0000 R CNN
F 1 "LED" V 6097 4183 50  0000 R CNN
F 2 "LEDs:LED_D3.0mm" H 6150 4300 50  0001 C CNN
F 3 "" H 6150 4300 50  0001 C CNN
	1    6150 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 6450 7600 5800
Wire Wire Line
	7600 4400 7600 5800
Connection ~ 7600 5800
Wire Wire Line
	6150 5800 6150 4750
Wire Wire Line
	6400 4750 6400 5800
Connection ~ 6400 5800
Wire Wire Line
	6400 5800 6150 5800
Wire Wire Line
	7600 5800 6600 5800
Wire Wire Line
	6600 5800 6400 5800
Connection ~ 6600 5800
Wire Wire Line
	6600 4750 6600 5800
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
	5700 7000 6750 7000
Wire Wire Line
	9500 2700 9350 2700
Wire Wire Line
	9900 2900 9500 2900
Wire Wire Line
	9500 2900 9500 2700
$EndSCHEMATC
