EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32f103
LIBS:bluepill_breakouts
LIBS:W5500
LIBS:93c46
LIBS:WIZnet_W5500_MagJack-cache
LIBS:WIZ850io
LIBS:bluepill_ethernet-cache
EELAYER 25 0
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
L BluePill_STM32F103C U1
U 1 1 5ABD20EB
P 3400 3250
F 0 "U1" H 2950 1900 50  0000 C CNN
F 1 "BluePill_STM32F103C" H 3450 4100 50  0000 C CNN
F 2 "BluePill_breakouts:BluePill_STM32F103C" H 3450 1650 50  0001 C CNN
F 3 "www.rogerclark.net" H 3400 1750 50  0001 C CNN
	1    3400 3250
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR01
U 1 1 5ABD2688
P 4900 900
F 0 "#PWR01" H 4900 750 50  0001 C CNN
F 1 "VCC" H 4900 1050 50  0000 C CNN
F 2 "" H 4900 900 50  0001 C CNN
F 3 "" H 4900 900 50  0001 C CNN
	1    4900 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5ABD2774
P 5600 1500
F 0 "#PWR02" H 5600 1250 50  0001 C CNN
F 1 "GND" H 5600 1350 50  0000 C CNN
F 2 "" H 5600 1500 50  0001 C CNN
F 3 "" H 5600 1500 50  0001 C CNN
	1    5600 1500
	1    0    0    -1  
$EndComp
$Comp
L 93C46 U2
U 1 1 5ABD4EFE
P 6350 3700
F 0 "U2" H 6350 3950 60  0000 C CNN
F 1 "93C46" H 6350 3425 60  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 6350 3700 60  0001 C CNN
F 3 "" H 6350 3700 60  0001 C CNN
	1    6350 3700
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J2
U 1 1 5ABD688D
P 4550 6250
F 0 "J2" H 4550 6450 50  0000 C CNN
F 1 "Conn_01x03" H 4550 6050 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-03_03x2.54mm_Straight" H 4550 6250 50  0001 C CNN
F 3 "" H 4550 6250 50  0001 C CNN
	1    4550 6250
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 5ABD825B
P 4250 5650
F 0 "R1" V 4330 5650 50  0000 C CNN
F 1 "R" V 4250 5650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 4180 5650 50  0001 C CNN
F 3 "" H 4250 5650 50  0001 C CNN
	1    4250 5650
	1    0    0    -1  
$EndComp
$Comp
L WIZ850io U3
U 1 1 5ABD9273
P 1550 2700
F 0 "U3" H 1550 2300 60  0000 C CNN
F 1 "WIZ850io" H 1550 3100 60  0000 C CNN
F 2 "kicad:WIZ850io" H 1450 2600 60  0001 C CNN
F 3 "" H 1450 2600 60  0001 C CNN
	1    1550 2700
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5ABDB1C7
P 1550 2100
F 0 "C1" H 1575 2200 50  0000 L CNN
F 1 "C" H 1575 2000 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 1588 1950 50  0001 C CNN
F 3 "" H 1550 2100 50  0001 C CNN
	1    1550 2100
	0    1    1    0   
$EndComp
$Comp
L C C5
U 1 1 5ABDB608
P 7300 3700
F 0 "C5" H 7325 3800 50  0000 L CNN
F 1 "C" H 7325 3600 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 7338 3550 50  0001 C CNN
F 3 "" H 7300 3700 50  0001 C CNN
	1    7300 3700
	-1   0    0    1   
$EndComp
$Comp
L C C3
U 1 1 5ABDBC12
P 4850 5650
F 0 "C3" H 4875 5750 50  0000 L CNN
F 1 "C" H 4875 5550 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 4888 5500 50  0001 C CNN
F 3 "" H 4850 5650 50  0001 C CNN
	1    4850 5650
	1    0    0    -1  
$EndComp
$Comp
L CP C4
U 1 1 5ABDC0B1
P 5100 5650
F 0 "C4" H 5125 5750 50  0000 L CNN
F 1 "CP" H 5125 5550 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 5138 5500 50  0001 C CNN
F 3 "" H 5100 5650 50  0001 C CNN
	1    5100 5650
	1    0    0    -1  
$EndComp
$Comp
L CP C2
U 1 1 5ABDC134
P 3750 1050
F 0 "C2" H 3775 1150 50  0000 L CNN
F 1 "CP" H 3775 950 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 3788 900 50  0001 C CNN
F 3 "" H 3750 1050 50  0001 C CNN
	1    3750 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4150 2550 4050
Connection ~ 4450 5500
Wire Wire Line
	3450 5400 4450 5400
Wire Wire Line
	3450 2850 3450 5400
Wire Wire Line
	2550 2850 3450 2850
Wire Wire Line
	2350 3350 2550 3350
Wire Wire Line
	2350 2850 2350 3350
Wire Wire Line
	2050 2850 2350 2850
Wire Wire Line
	2500 3750 2550 3750
Wire Wire Line
	2500 4950 2500 3750
Wire Wire Line
	4750 4950 2500 4950
Wire Wire Line
	4750 3550 4750 4950
Wire Wire Line
	5800 3550 4750 3550
Connection ~ 6900 3850
Wire Wire Line
	6900 3750 6900 3850
Connection ~ 1050 2100
Wire Wire Line
	1400 2100 1050 2100
Connection ~ 2150 2100
Wire Wire Line
	1700 2100 2150 2100
Connection ~ 3750 1200
Connection ~ 3750 900 
Connection ~ 4550 5850
Wire Wire Line
	5600 1200 5600 1500
Wire Wire Line
	3050 900  2150 1000
Wire Wire Line
	3050 900  6900 900 
Wire Wire Line
	4300 900  4300 2550
Connection ~ 4900 900 
Wire Wire Line
	5100 3750 5100 5200
Wire Wire Line
	5100 5200 2200 5200
Wire Wire Line
	2200 5200 2200 3650
Wire Wire Line
	750  3650 2550 3650
Wire Wire Line
	4900 3650 4900 5100
Wire Wire Line
	4900 5100 2350 5100
Wire Wire Line
	2350 5100 2350 3450
Wire Wire Line
	850  3450 2550 3450
Wire Wire Line
	2050 5300 5450 5300
Wire Wire Line
	2050 2950 2050 5300
Wire Wire Line
	2050 3550 2550 3550
Connection ~ 4300 900 
Wire Wire Line
	1050 1200 7100 1200
Wire Wire Line
	4450 2650 4300 2650
Wire Wire Line
	4450 1200 4450 2650
Connection ~ 4450 1200
Wire Wire Line
	2550 4350 2550 4550
Wire Wire Line
	2550 4550 3200 4550
Wire Wire Line
	3200 4550 3200 1200
Connection ~ 2550 4450
Wire Wire Line
	2550 4250 3050 4250
Wire Wire Line
	3050 4250 3050 900 
Wire Wire Line
	6900 900  6900 3550
Wire Wire Line
	6900 3850 7300 3850
Wire Wire Line
	7100 1200 7100 3850
Connection ~ 5600 1200
Wire Wire Line
	5800 3650 4900 3650
Wire Wire Line
	5800 3750 5100 3750
Wire Wire Line
	5800 3850 5450 3850
Wire Wire Line
	5450 3850 5450 5300
Wire Wire Line
	4450 5400 4450 6050
Wire Wire Line
	4550 1200 4550 6050
Connection ~ 4550 1200
Wire Wire Line
	4650 900  4650 6050
Connection ~ 4650 900 
Wire Wire Line
	4250 5500 4450 5500
Wire Wire Line
	4650 5800 4250 5800
Connection ~ 4650 5800
Wire Wire Line
	1050 1200 1050 2550
Connection ~ 3200 1200
Connection ~ 1050 2450
Wire Wire Line
	2050 2450 2050 1200
Connection ~ 2050 1200
Wire Wire Line
	2050 2550 2150 2550
Wire Wire Line
	2150 1000 2150 2650
Connection ~ 3050 900 
Wire Wire Line
	2150 2650 2050 2650
Connection ~ 2150 2550
Connection ~ 2050 3550
Wire Wire Line
	850  3450 850  2750
Wire Wire Line
	850  2750 1050 2750
Connection ~ 2350 3450
Wire Wire Line
	750  3650 750  2650
Wire Wire Line
	750  2650 1050 2650
Connection ~ 2200 3650
Wire Wire Line
	4650 5500 5100 5500
Connection ~ 4850 5500
Connection ~ 4650 5500
Wire Wire Line
	4550 5850 5100 5850
Wire Wire Line
	4850 5850 4850 5800
Wire Wire Line
	5100 5850 5100 5800
Connection ~ 4850 5850
Connection ~ 7100 3850
Wire Wire Line
	6900 3550 7300 3550
Wire Wire Line
	1050 2950 1050 3950
Wire Wire Line
	1050 3950 2550 3950
Wire Wire Line
	2550 3850 950  3850
Wire Wire Line
	950  3850 950  2850
Wire Wire Line
	950  2850 1050 2850
$EndSCHEMATC
