EESchema Schematic File Version 4
LIBS:RC-Endpoint-Limiter-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "RC Endpoint Limiter"
Date "2019-09-01"
Rev "0.01"
Comp "Vapourforge"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:Crystal_Small Y1
U 1 1 5D475521
P 3200 3200
F 0 "Y1" V 3154 3288 50  0000 L CNN
F 1 "8Mhz" V 3245 3288 50  0000 L CNN
F 2 "" H 3200 3200 50  0001 C CNN
F 3 "~" H 3200 3200 50  0001 C CNN
	1    3200 3200
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5D476312
P 2750 3100
F 0 "C1" V 2700 3000 50  0000 C CNN
F 1 "22pf" V 2700 3300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2750 3100 50  0001 C CNN
F 3 "~" H 2750 3100 50  0001 C CNN
	1    2750 3100
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5D477709
P 2750 3300
F 0 "C2" V 2800 3400 50  0000 C CNN
F 1 "22pf" V 2800 3100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2750 3300 50  0001 C CNN
F 3 "~" H 2750 3300 50  0001 C CNN
	1    2750 3300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3650 3100 3200 3100
Connection ~ 3200 3100
Wire Wire Line
	3650 3300 3200 3300
Connection ~ 3200 3300
Wire Wire Line
	4350 2600 4350 2450
$Comp
L Device:C_Small C3
U 1 1 5D478F2C
P 2750 3500
F 0 "C3" V 2700 3400 50  0000 C CNN
F 1 "100n" V 2700 3700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2750 3500 50  0001 C CNN
F 3 "~" H 2750 3500 50  0001 C CNN
	1    2750 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	2850 3100 3200 3100
Wire Wire Line
	2850 3300 3200 3300
Wire Wire Line
	2850 3500 3650 3500
$Comp
L power:GND #PWR04
U 1 1 5D47C8B5
P 2200 3400
F 0 "#PWR04" H 2200 3150 50  0001 C CNN
F 1 "GND" H 2205 3227 50  0000 C CNN
F 2 "" H 2200 3400 50  0001 C CNN
F 3 "" H 2200 3400 50  0001 C CNN
	1    2200 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3500 2450 3500
Wire Wire Line
	2650 3300 2450 3300
Wire Wire Line
	2450 3300 2450 3500
Wire Wire Line
	2650 3100 2450 3100
Wire Wire Line
	2450 3100 2450 3300
Connection ~ 2450 3300
$Comp
L power:VCC #PWR05
U 1 1 5D47DEA1
P 2450 2750
F 0 "#PWR05" H 2450 2600 50  0001 C CNN
F 1 "VCC" H 2467 2923 50  0000 C CNN
F 2 "" H 2450 2750 50  0001 C CNN
F 3 "" H 2450 2750 50  0001 C CNN
	1    2450 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5D47F0DD
P 3200 2900
F 0 "R3" V 3150 2750 50  0000 C CNN
F 1 "10K" V 3150 3050 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3200 2900 50  0001 C CNN
F 3 "~" H 3200 2900 50  0001 C CNN
	1    3200 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	3650 2900 3300 2900
Wire Wire Line
	2450 2750 2450 2900
Wire Wire Line
	2450 2900 3100 2900
$Comp
L power:GND #PWR09
U 1 1 5D4808EF
P 5050 2500
F 0 "#PWR09" H 5050 2250 50  0001 C CNN
F 1 "GND" H 5055 2327 50  0000 C CNN
F 2 "" H 5050 2500 50  0001 C CNN
F 3 "" H 5050 2500 50  0001 C CNN
	1    5050 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5D481418
P 4600 2450
F 0 "C7" V 4550 2350 50  0000 C CNN
F 1 "1uF" V 4550 2650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4600 2450 50  0001 C CNN
F 3 "~" H 4600 2450 50  0001 C CNN
	1    4600 2450
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5D48266B
P 4600 2300
F 0 "C6" V 4550 2200 50  0000 C CNN
F 1 "1uF" V 4550 2500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4600 2300 50  0001 C CNN
F 3 "~" H 4600 2300 50  0001 C CNN
	1    4600 2300
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5D482F2D
P 4600 2150
F 0 "C5" V 4550 2050 50  0000 C CNN
F 1 "1uF" V 4550 2350 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4600 2150 50  0001 C CNN
F 3 "~" H 4600 2150 50  0001 C CNN
	1    4600 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	4700 2450 5050 2450
Wire Wire Line
	5050 2450 5050 2500
Wire Wire Line
	4700 2300 5050 2300
Wire Wire Line
	5050 2300 5050 2450
Connection ~ 5050 2450
Wire Wire Line
	4700 2150 5050 2150
Wire Wire Line
	5050 2150 5050 2300
Connection ~ 5050 2300
Wire Wire Line
	4150 2150 4500 2150
Wire Wire Line
	4250 2300 4500 2300
Wire Wire Line
	4350 2450 4500 2450
$Comp
L power:VCC #PWR08
U 1 1 5D487471
P 4250 1650
F 0 "#PWR08" H 4250 1500 50  0001 C CNN
F 1 "VCC" H 4267 1823 50  0000 C CNN
F 2 "" H 4250 1650 50  0001 C CNN
F 3 "" H 4250 1650 50  0001 C CNN
	1    4250 1650
	1    0    0    -1  
$EndComp
Connection ~ 4350 2450
$Comp
L Device:C_Small C4
U 1 1 5D488BA8
P 3300 4200
F 0 "C4" V 3250 4100 50  0000 C CNN
F 1 "1uF" V 3250 4400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3300 4200 50  0001 C CNN
F 3 "~" H 3300 4200 50  0001 C CNN
	1    3300 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	3650 4200 3400 4200
$Comp
L power:GND #PWR06
U 1 1 5D489F30
P 3150 4250
F 0 "#PWR06" H 3150 4000 50  0001 C CNN
F 1 "GND" H 3155 4077 50  0000 C CNN
F 2 "" H 3150 4250 50  0001 C CNN
F 3 "" H 3150 4250 50  0001 C CNN
	1    3150 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4200 3150 4200
Wire Wire Line
	3150 4200 3150 4250
Wire Wire Line
	2450 3300 2200 3300
Wire Wire Line
	2200 3300 2200 3400
Wire Wire Line
	3650 3700 1950 3700
$Comp
L Connector:USB_B_Micro J1
U 1 1 5D4913BF
P 1550 3900
F 0 "J1" H 1607 4367 50  0000 C CNN
F 1 "USB_B_Micro" H 1607 4276 50  0000 C CNN
F 2 "" H 1700 3850 50  0001 C CNN
F 3 "~" H 1700 3850 50  0001 C CNN
	1    1550 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5D49051B
P 1550 4450
F 0 "#PWR02" H 1550 4200 50  0001 C CNN
F 1 "GND" H 1555 4277 50  0000 C CNN
F 2 "" H 1550 4450 50  0001 C CNN
F 3 "" H 1550 4450 50  0001 C CNN
	1    1550 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 4300 1550 4450
$Comp
L Device:R_Small R1
U 1 1 5D4916F0
P 2250 3900
F 0 "R1" V 2200 3750 50  0000 C CNN
F 1 "22R" V 2200 4050 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2250 3900 50  0001 C CNN
F 3 "~" H 2250 3900 50  0001 C CNN
	1    2250 3900
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5D491BDA
P 2250 4000
F 0 "R2" V 2200 3850 50  0000 C CNN
F 1 "22R" V 2200 4150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2250 4000 50  0001 C CNN
F 3 "~" H 2250 4000 50  0001 C CNN
	1    2250 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	1850 3900 2150 3900
Wire Wire Line
	1850 4000 2150 4000
Wire Wire Line
	2350 4000 3650 4000
Wire Wire Line
	3650 3900 2350 3900
$Comp
L power:VCC #PWR03
U 1 1 5D6A1239
P 1950 2750
F 0 "#PWR03" H 1950 2600 50  0001 C CNN
F 1 "VCC" H 1967 2923 50  0000 C CNN
F 2 "" H 1950 2750 50  0001 C CNN
F 3 "" H 1950 2750 50  0001 C CNN
	1    1950 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3200 1950 3700
Connection ~ 1950 3700
Wire Wire Line
	1950 3700 1850 3700
Wire Wire Line
	1950 2750 1950 3000
$Comp
L Device:LED_Small D2
U 1 1 5D6A916E
P 6050 5300
F 0 "D2" H 6150 5250 50  0000 C CNN
F 1 "CPU" H 5900 5250 50  0000 C CNN
F 2 "" V 6050 5300 50  0001 C CNN
F 3 "~" V 6050 5300 50  0001 C CNN
	1    6050 5300
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D3
U 1 1 5D6AA2EB
P 6050 5400
F 0 "D3" H 6150 5350 50  0000 C CNN
F 1 "Signal" H 5850 5350 50  0000 C CNN
F 2 "" V 6050 5400 50  0001 C CNN
F 3 "~" V 6050 5400 50  0001 C CNN
	1    6050 5400
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D4
U 1 1 5D6AA52C
P 6050 5500
F 0 "D4" H 6150 5450 50  0000 C CNN
F 1 "Low" H 5825 5450 50  0000 L CNN
F 2 "" V 6050 5500 50  0001 C CNN
F 3 "~" V 6050 5500 50  0001 C CNN
	1    6050 5500
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D5
U 1 1 5D6AA8EC
P 6050 5600
F 0 "D5" H 6150 5550 50  0000 C CNN
F 1 "High" H 5800 5550 50  0000 L CNN
F 2 "" V 6050 5600 50  0001 C CNN
F 3 "~" V 6050 5600 50  0001 C CNN
	1    6050 5600
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R11
U 1 1 5D6AC123
P 5550 5300
F 0 "R11" V 5500 5150 50  0000 C CNN
F 1 "240R" V 5500 5500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 5300 50  0001 C CNN
F 3 "~" H 5550 5300 50  0001 C CNN
	1    5550 5300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R12
U 1 1 5D6AF034
P 5550 5400
F 0 "R12" V 5500 5250 50  0000 C CNN
F 1 "240R" V 5500 5600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 5400 50  0001 C CNN
F 3 "~" H 5550 5400 50  0001 C CNN
	1    5550 5400
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R13
U 1 1 5D6AF211
P 5550 5500
F 0 "R13" V 5500 5350 50  0000 C CNN
F 1 "240R" V 5500 5700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 5500 50  0001 C CNN
F 3 "~" H 5550 5500 50  0001 C CNN
	1    5550 5500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R14
U 1 1 5D6AF4CB
P 5550 5600
F 0 "R14" V 5500 5450 50  0000 C CNN
F 1 "240R" V 5500 5800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 5600 50  0001 C CNN
F 3 "~" H 5550 5600 50  0001 C CNN
	1    5550 5600
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 5600 5950 5600
Wire Wire Line
	5650 5500 5950 5500
Wire Wire Line
	5650 5400 5950 5400
Wire Wire Line
	5650 5300 5950 5300
$Comp
L power:GND #PWR07
U 1 1 5D6B7D97
P 4200 6450
F 0 "#PWR07" H 4200 6200 50  0001 C CNN
F 1 "GND" H 4205 6277 50  0000 C CNN
F 2 "" H 4200 6450 50  0001 C CNN
F 3 "" H 4200 6450 50  0001 C CNN
	1    4200 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 6200 4150 6450
Wire Wire Line
	4150 6450 4200 6450
Wire Wire Line
	4250 6200 4250 6450
Wire Wire Line
	4250 6450 4200 6450
Connection ~ 4200 6450
$Comp
L power:GND #PWR014
U 1 1 5D6BA7FA
P 6450 5650
F 0 "#PWR014" H 6450 5400 50  0001 C CNN
F 1 "GND" H 6455 5477 50  0000 C CNN
F 2 "" H 6450 5650 50  0001 C CNN
F 3 "" H 6450 5650 50  0001 C CNN
	1    6450 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 5600 6450 5650
Wire Wire Line
	6450 5500 6450 5600
Connection ~ 6450 5600
Wire Wire Line
	6450 5400 6450 5500
Connection ~ 6450 5500
Wire Wire Line
	6450 5300 6450 5400
Connection ~ 6450 5400
Wire Wire Line
	6150 5600 6450 5600
Wire Wire Line
	6150 5500 6450 5500
Wire Wire Line
	6150 5400 6450 5400
Wire Wire Line
	6150 5300 6450 5300
$Comp
L Device:R_Small R10
U 1 1 5D6CDAD2
P 5550 5000
F 0 "R10" V 5500 4850 50  0000 C CNN
F 1 "10K" V 5500 5150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 5000 50  0001 C CNN
F 3 "~" H 5550 5000 50  0001 C CNN
	1    5550 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 5000 6450 5000
Wire Wire Line
	4350 1750 4350 2450
$Comp
L Connector_Generic:Conn_01x03 J7
U 1 1 5D6D9B66
P 9550 2450
F 0 "J7" V 9514 2262 50  0000 R CNN
F 1 "RC Output" V 9700 2600 50  0000 R CNN
F 2 "" H 9550 2450 50  0001 C CNN
F 3 "~" H 9550 2450 50  0001 C CNN
	1    9550 2450
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR022
U 1 1 5D6DA979
P 9250 1650
F 0 "#PWR022" H 9250 1500 50  0001 C CNN
F 1 "VCC" H 9267 1823 50  0000 C CNN
F 2 "" H 9250 1650 50  0001 C CNN
F 3 "" H 9250 1650 50  0001 C CNN
	1    9250 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5D6DB42E
P 9450 2750
F 0 "#PWR023" H 9450 2500 50  0001 C CNN
F 1 "GND" H 9455 2577 50  0000 C CNN
F 2 "" H 9450 2750 50  0001 C CNN
F 3 "" H 9450 2750 50  0001 C CNN
	1    9450 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 2650 9450 2750
Wire Wire Line
	9550 2650 9550 3000
Wire Wire Line
	9550 3000 9250 3000
Wire Wire Line
	9250 1800 9250 1650
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 5D6F7473
P 10250 2450
F 0 "J6" V 10214 2262 50  0000 R CNN
F 1 "RC Input" V 10400 2600 50  0000 R CNN
F 2 "" H 10250 2450 50  0001 C CNN
F 3 "~" H 10250 2450 50  0001 C CNN
	1    10250 2450
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR020
U 1 1 5D6F7479
P 9950 1650
F 0 "#PWR020" H 9950 1500 50  0001 C CNN
F 1 "VCC" H 9967 1823 50  0000 C CNN
F 2 "" H 9950 1650 50  0001 C CNN
F 3 "" H 9950 1650 50  0001 C CNN
	1    9950 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5D6F747F
P 10150 2750
F 0 "#PWR021" H 10150 2500 50  0001 C CNN
F 1 "GND" H 10155 2577 50  0000 C CNN
F 2 "" H 10150 2750 50  0001 C CNN
F 3 "" H 10150 2750 50  0001 C CNN
	1    10150 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 2650 10150 2750
Wire Wire Line
	10250 2650 10250 3000
Wire Wire Line
	10250 3000 9950 3000
Wire Wire Line
	9950 1800 9950 1650
$Comp
L Connector:Conn_01x02_Female J4
U 1 1 5D712208
P 7350 2300
F 0 "J4" V 7288 2112 50  0000 R CNN
F 1 "Switch Low" V 7400 2450 50  0000 R CNN
F 2 "Connector_AMASS:AMASS_MR30PW-FB_1x03_P3.50mm_Horizontal" H 7350 2300 50  0001 C CNN
F 3 "~" H 7350 2300 50  0001 C CNN
	1    7350 2300
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x02_Female J5
U 1 1 5D7134E8
P 8300 2300
F 0 "J5" V 8238 2112 50  0000 R CNN
F 1 "Switch High" V 8350 2450 50  0000 R CNN
F 2 "Connector_AMASS:AMASS_MR30PW-FB_1x03_P3.50mm_Horizontal" H 8300 2300 50  0001 C CNN
F 3 "~" H 8300 2300 50  0001 C CNN
	1    8300 2300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10350 4300 10350 2650
Wire Wire Line
	9650 2650 9650 3800
$Comp
L Device:R_Small R15
U 1 1 5D7396D9
P 7100 1900
F 0 "R15" V 7050 1750 50  0000 C CNN
F 1 "240R" V 7050 2100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7100 1900 50  0001 C CNN
F 3 "~" H 7100 1900 50  0001 C CNN
	1    7100 1900
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5D73F345
P 5550 3500
F 0 "R4" V 5500 3350 50  0000 C CNN
F 1 "1K" V 5500 3700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 3500 50  0001 C CNN
F 3 "~" H 5550 3500 50  0001 C CNN
	1    5550 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5D74031D
P 5550 4200
F 0 "R6" V 5500 4050 50  0000 C CNN
F 1 "1K" V 5500 4400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 4200 50  0001 C CNN
F 3 "~" H 5550 4200 50  0001 C CNN
	1    5550 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R9
U 1 1 5D74838A
P 5550 4500
F 0 "R9" V 5500 4350 50  0000 C CNN
F 1 "1K" V 5500 4700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 4500 50  0001 C CNN
F 3 "~" H 5550 4500 50  0001 C CNN
	1    5550 4500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5D74AF8A
P 5550 3800
F 0 "R5" V 5500 3650 50  0000 C CNN
F 1 "100R" V 5500 4000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 3800 50  0001 C CNN
F 3 "~" H 5550 3800 50  0001 C CNN
	1    5550 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 3800 6000 3800
$Comp
L Device:R_Small R16
U 1 1 5D7602C0
P 7700 2650
F 0 "R16" V 7650 2500 50  0000 C CNN
F 1 "1K" V 7650 2850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7700 2650 50  0001 C CNN
F 3 "~" H 7700 2650 50  0001 C CNN
	1    7700 2650
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R18
U 1 1 5D7621D5
P 8650 2650
F 0 "R18" V 8600 2500 50  0000 C CNN
F 1 "1K" V 8600 2850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8650 2650 50  0001 C CNN
F 3 "~" H 8650 2650 50  0001 C CNN
	1    8650 2650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5D76620A
P 8850 2750
F 0 "#PWR019" H 8850 2500 50  0001 C CNN
F 1 "GND" H 8855 2577 50  0000 C CNN
F 2 "" H 8850 2750 50  0001 C CNN
F 3 "" H 8850 2750 50  0001 C CNN
	1    8850 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5D7665F3
P 7900 2750
F 0 "#PWR017" H 7900 2500 50  0001 C CNN
F 1 "GND" H 7905 2577 50  0000 C CNN
F 2 "" H 7900 2750 50  0001 C CNN
F 3 "" H 7900 2750 50  0001 C CNN
	1    7900 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2650 7900 2650
Wire Wire Line
	7900 2650 7900 2750
Wire Wire Line
	8750 2650 8850 2650
Wire Wire Line
	8850 2650 8850 2750
Wire Wire Line
	8550 2650 8400 2650
Wire Wire Line
	8400 2650 8400 2500
Wire Wire Line
	7600 2650 7450 2650
Wire Wire Line
	7450 2650 7450 2500
Wire Wire Line
	7350 2500 7350 2650
Wire Wire Line
	7350 2650 7100 2650
Wire Wire Line
	8300 2500 8300 2650
Wire Wire Line
	8300 2650 8050 2650
$Comp
L power:VCC #PWR018
U 1 1 5D77627B
P 8050 1650
F 0 "#PWR018" H 8050 1500 50  0001 C CNN
F 1 "VCC" H 8067 1823 50  0000 C CNN
F 2 "" H 8050 1650 50  0001 C CNN
F 3 "" H 8050 1650 50  0001 C CNN
	1    8050 1650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR016
U 1 1 5D776C26
P 7100 1650
F 0 "#PWR016" H 7100 1500 50  0001 C CNN
F 1 "VCC" H 7117 1823 50  0000 C CNN
F 2 "" H 7100 1650 50  0001 C CNN
F 3 "" H 7100 1650 50  0001 C CNN
	1    7100 1650
	1    0    0    -1  
$EndComp
Connection ~ 7450 2650
Connection ~ 8400 2650
$Comp
L Device:R_Small R17
U 1 1 5D73AC12
P 8050 1900
F 0 "R17" V 8000 1750 50  0000 C CNN
F 1 "240R" V 8000 2100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8050 1900 50  0001 C CNN
F 3 "~" H 8050 1900 50  0001 C CNN
	1    8050 1900
	-1   0    0    1   
$EndComp
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 5D7A4F8C
P 9600 2150
F 0 "JP1" H 9600 2355 50  0000 C CNN
F 1 "PassThrough" H 9600 2264 50  0000 C CNN
F 2 "" H 9600 2150 50  0001 C CNN
F 3 "~" H 9600 2150 50  0001 C CNN
	1    9600 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 2000 7100 2650
Wire Wire Line
	7100 1800 7100 1650
Wire Wire Line
	8050 2000 8050 2650
$Comp
L Device:D_Schottky_Small D1
U 1 1 5D7D43AA
P 1950 3100
F 0 "D1" V 1904 3032 50  0000 R CNN
F 1 "USB Supply" V 1995 3032 50  0000 R CNN
F 2 "" V 1950 3100 50  0001 C CNN
F 3 "~" V 1950 3100 50  0001 C CNN
	1    1950 3100
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky_Small D7
U 1 1 5D7DA818
P 9250 1900
F 0 "D7" V 9204 1833 50  0000 R CNN
F 1 "Output Supply" V 9295 1833 50  0000 R CNN
F 2 "" V 9250 1900 50  0001 C CNN
F 3 "~" V 9250 1900 50  0001 C CNN
	1    9250 1900
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky_Small D6
U 1 1 5D7DC69D
P 9950 1900
F 0 "D6" V 9904 1968 50  0000 L CNN
F 1 "Input Supply" V 9995 1968 50  0000 L CNN
F 2 "" V 9950 1900 50  0001 C CNN
F 3 "~" V 9950 1900 50  0001 C CNN
	1    9950 1900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5D7E4AFB
P 6450 5050
F 0 "#PWR013" H 6450 4800 50  0001 C CNN
F 1 "GND" H 6455 4877 50  0000 C CNN
F 2 "" H 6450 5050 50  0001 C CNN
F 3 "" H 6450 5050 50  0001 C CNN
	1    6450 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 5050 6450 5000
NoConn ~ 4850 2900
NoConn ~ 4850 3000
NoConn ~ 4850 3100
NoConn ~ 4850 3600
NoConn ~ 4850 3900
NoConn ~ 4850 4100
NoConn ~ 4850 4600
NoConn ~ 4850 4700
NoConn ~ 4850 4800
NoConn ~ 4850 5100
NoConn ~ 4850 5700
NoConn ~ 4850 5800
Wire Wire Line
	4150 1750 4250 1750
Wire Wire Line
	4250 1750 4250 1650
Connection ~ 4250 1750
Wire Wire Line
	4250 1750 4350 1750
Wire Wire Line
	4250 1750 4250 2300
Wire Wire Line
	4250 2300 4250 2600
Connection ~ 4250 2300
Wire Wire Line
	4150 1750 4150 2150
Wire Wire Line
	4150 2150 4150 2600
Connection ~ 4150 2150
Wire Wire Line
	8050 1650 8050 1800
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5D70BE31
P 1050 1100
F 0 "#FLG01" H 1050 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 1050 1273 50  0000 C CNN
F 2 "" H 1050 1100 50  0001 C CNN
F 3 "~" H 1050 1100 50  0001 C CNN
	1    1050 1100
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR01
U 1 1 5D70C0E6
P 1350 1100
F 0 "#PWR01" H 1350 950 50  0001 C CNN
F 1 "VCC" H 1367 1273 50  0000 C CNN
F 2 "" H 1350 1100 50  0001 C CNN
F 3 "" H 1350 1100 50  0001 C CNN
	1    1350 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 1100 1050 1200
Wire Wire Line
	1050 1200 1350 1200
Wire Wire Line
	1350 1200 1350 1100
NoConn ~ 1850 4100
NoConn ~ 1450 4300
Wire Wire Line
	4850 5000 5450 5000
Wire Wire Line
	4850 5300 5450 5300
Wire Wire Line
	4850 5400 5450 5400
Wire Wire Line
	4850 5500 5450 5500
Wire Wire Line
	4850 5600 5450 5600
Wire Wire Line
	4850 3800 5450 3800
$Comp
L Device:R_Small R8
U 1 1 5D789E83
P 5550 4400
F 0 "R8" V 5500 4250 50  0000 C CNN
F 1 "100R" V 5500 4600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 4400 50  0001 C CNN
F 3 "~" H 5550 4400 50  0001 C CNN
	1    5550 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	4850 4400 5450 4400
Wire Wire Line
	5650 4400 6000 4400
Wire Wire Line
	6000 4400 6000 3800
$Comp
L Device:R_Small R7
U 1 1 5D792A3F
P 5550 4300
F 0 "R7" V 5500 4150 50  0000 C CNN
F 1 "1K" V 5500 4500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 4300 50  0001 C CNN
F 3 "~" H 5550 4300 50  0001 C CNN
	1    5550 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	4850 4300 5450 4300
Wire Wire Line
	5650 4300 6150 4300
Wire Wire Line
	4850 4500 5450 4500
Wire Wire Line
	5650 4500 6150 4500
Wire Wire Line
	6150 4500 6150 4300
Wire Wire Line
	4850 4200 5450 4200
Wire Wire Line
	8400 2650 8400 4200
Wire Wire Line
	5650 4200 8400 4200
Connection ~ 6150 4300
Wire Wire Line
	6000 3800 9650 3800
Connection ~ 6000 3800
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 5D81C652
P 6050 2450
F 0 "J2" V 6014 2262 50  0000 R CNN
F 1 "Aux Low" V 6200 2600 50  0000 R CNN
F 2 "" H 6050 2450 50  0001 C CNN
F 3 "~" H 6050 2450 50  0001 C CNN
	1    6050 2450
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 5D81DE9D
P 6650 2450
F 0 "J3" V 6614 2262 50  0000 R CNN
F 1 "Aux High" V 6800 2600 50  0000 R CNN
F 2 "" H 6650 2450 50  0001 C CNN
F 3 "~" H 6650 2450 50  0001 C CNN
	1    6650 2450
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR010
U 1 1 5D83710C
P 5850 1650
F 0 "#PWR010" H 5850 1500 50  0001 C CNN
F 1 "VCC" H 5867 1823 50  0000 C CNN
F 2 "" H 5850 1650 50  0001 C CNN
F 3 "" H 5850 1650 50  0001 C CNN
	1    5850 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5D8379C3
P 6050 2750
F 0 "#PWR011" H 6050 2500 50  0001 C CNN
F 1 "GND" H 6055 2577 50  0000 C CNN
F 2 "" H 6050 2750 50  0001 C CNN
F 3 "" H 6050 2750 50  0001 C CNN
	1    6050 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2700 5950 2700
Wire Wire Line
	5950 2700 5950 2650
Wire Wire Line
	6050 2750 6050 2650
Wire Wire Line
	4850 3300 6150 3300
Wire Wire Line
	6150 3300 6150 2650
Wire Wire Line
	4850 3400 6750 3400
Wire Wire Line
	6750 3400 6750 2650
$Comp
L power:GND #PWR015
U 1 1 5D84E821
P 6650 2750
F 0 "#PWR015" H 6650 2500 50  0001 C CNN
F 1 "GND" H 6655 2577 50  0000 C CNN
F 2 "" H 6650 2750 50  0001 C CNN
F 3 "" H 6650 2750 50  0001 C CNN
	1    6650 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 2650 6650 2750
$Comp
L power:VCC #PWR012
U 1 1 5D85377E
P 6450 1650
F 0 "#PWR012" H 6450 1500 50  0001 C CNN
F 1 "VCC" H 6467 1823 50  0000 C CNN
F 2 "" H 6450 1650 50  0001 C CNN
F 3 "" H 6450 1650 50  0001 C CNN
	1    6450 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2650 6550 2700
Wire Wire Line
	6550 2700 6450 2700
Wire Wire Line
	5850 1650 5850 2700
Wire Wire Line
	6450 1650 6450 2700
Wire Wire Line
	4850 3500 5450 3500
Wire Wire Line
	7450 2650 7450 3500
Wire Wire Line
	5650 3500 7450 3500
NoConn ~ 4850 3200
Wire Wire Line
	6150 4300 10350 4300
Wire Wire Line
	9250 2000 9250 2150
Wire Wire Line
	9950 2000 9950 2150
Wire Wire Line
	9450 2150 9250 2150
Connection ~ 9250 2150
Wire Wire Line
	9250 2150 9250 3000
Wire Wire Line
	9750 2150 9950 2150
Connection ~ 9950 2150
Wire Wire Line
	9950 2150 9950 3000
$Comp
L MCU_Microchip_ATmega:ATmega32U4-MU U1
U 1 1 5D6CC858
P 4250 4400
F 0 "U1" H 4700 2650 50  0000 C CNN
F 1 "ATmega32U4-MU" H 4250 3600 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-44-1EP_7x7mm_P0.5mm_EP5.2x5.2mm" H 4250 4400 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf" H 4250 4400 50  0001 C CNN
	1    4250 4400
	1    0    0    -1  
$EndComp
$EndSCHEMATC
