EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr User 8000 6500
encoding utf-8
Sheet 1 1
Title "All-Terrain Buggy "
Date "2022-01-12"
Rev "v1.1"
Comp "M5 Makerspace "
Comment1 ""
Comment2 "creativecommons.org/licenses/by/4.0/"
Comment3 "CC BY 4.0"
Comment4 "Author: Akshat Sahay, Sebastian Armstrong "
$EndDescr
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 61DE8224
P 1100 1400
F 0 "J1" H 1018 1075 50  0000 C CNN
F 1 "XT60" H 1018 1166 50  0000 C CNN
F 2 "Connector_AMASS:AMASS_XT60-M_1x02_P7.20mm_Vertical" H 1100 1400 50  0001 C CNN
F 3 "~" H 1100 1400 50  0001 C CNN
	1    1100 1400
	-1   0    0    1   
$EndComp
$Comp
L power:VCC #PWR01
U 1 1 61DF6D62
P 1400 1300
F 0 "#PWR01" H 1400 1150 50  0001 C CNN
F 1 "VCC" H 1415 1473 50  0000 C CNN
F 2 "" H 1400 1300 50  0001 C CNN
F 3 "" H 1400 1300 50  0001 C CNN
	1    1400 1300
	1    0    0    -1  
$EndComp
Wire Notes Line
	1000 1000 1000 2000
Wire Wire Line
	1300 1300 1400 1300
Wire Wire Line
	1300 1400 1400 1400
Text Notes 1000 950  0    75   ~ 0
Power Electronics\n
NoConn ~ 2400 3700
NoConn ~ 2400 4200
NoConn ~ 2400 4100
NoConn ~ 2400 4000
NoConn ~ 1400 4200
NoConn ~ 1400 4000
NoConn ~ 1400 3800
NoConn ~ 1400 3700
Text Notes 5000 950  0    75   ~ 0
Control Electronics
Wire Notes Line
	5000 1000 5000 3500
Wire Notes Line
	5000 3500 7000 3500
Wire Notes Line
	7000 1000 5000 1000
Wire Notes Line
	7000 3500 7000 1000
Wire Notes Line
	1000 5500 1000 2500
NoConn ~ 1400 4800
NoConn ~ 1400 4700
NoConn ~ 1400 4500
NoConn ~ 1400 4400
NoConn ~ 2400 3500
NoConn ~ 2400 3400
NoConn ~ 2400 3100
NoConn ~ 2100 2700
NoConn ~ 2000 2700
NoConn ~ 1800 2700
$Comp
L power:GND #PWR0101
U 1 1 61D8BB98
P 1900 5200
F 0 "#PWR0101" H 1900 4950 50  0001 C CNN
F 1 "GND" H 1905 5027 50  0000 C CNN
F 2 "" H 1900 5200 50  0001 C CNN
F 3 "" H 1900 5200 50  0001 C CNN
	1    1900 5200
	1    0    0    -1  
$EndComp
Text GLabel 1400 3400 0    50   Input ~ 0
BIN2
Text GLabel 1400 3300 0    50   Input ~ 0
BIN1
Connection ~ 6300 3050
Wire Wire Line
	6300 3050 6300 3200
Connection ~ 5300 3050
Wire Wire Line
	5300 3050 5300 3200
$Comp
L power:GND #PWR05
U 1 1 61D3B442
P 6300 3200
F 0 "#PWR05" H 6300 2950 50  0001 C CNN
F 1 "GND" H 6305 3027 50  0000 C CNN
F 2 "" H 6300 3200 50  0001 C CNN
F 3 "" H 6300 3200 50  0001 C CNN
	1    6300 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 61D3A90D
P 5300 3200
F 0 "#PWR03" H 5300 2950 50  0001 C CNN
F 1 "GND" H 5305 3027 50  0000 C CNN
F 2 "" H 5300 3200 50  0001 C CNN
F 3 "" H 5300 3200 50  0001 C CNN
	1    5300 3200
	1    0    0    -1  
$EndComp
Text GLabel 6400 1450 0    50   Input ~ 0
BIN2
Text GLabel 6450 1350 0    50   Input ~ 0
BIN1
Wire Wire Line
	6300 2350 6300 3050
Connection ~ 6300 2350
Wire Wire Line
	6300 2350 6500 2350
Wire Wire Line
	6300 3050 6500 3050
Wire Wire Line
	6300 1650 6300 2350
Wire Wire Line
	6500 1650 6300 1650
Wire Wire Line
	6350 2250 6350 2950
Connection ~ 6350 2250
Wire Wire Line
	6350 2250 6500 2250
Wire Wire Line
	6350 2950 6500 2950
Wire Wire Line
	6350 1550 6350 2250
Wire Wire Line
	6500 1550 6350 1550
Connection ~ 6400 2150
Wire Wire Line
	6400 2850 6500 2850
Wire Wire Line
	6400 2150 6400 2850
Wire Wire Line
	6400 2150 6500 2150
Wire Wire Line
	6400 1450 6400 2150
Wire Wire Line
	6500 1450 6400 1450
Connection ~ 6450 2050
Wire Wire Line
	6450 2750 6500 2750
Wire Wire Line
	6450 2050 6450 2750
Wire Wire Line
	6450 2050 6500 2050
Wire Wire Line
	6450 1350 6450 2050
Wire Wire Line
	6500 1350 6450 1350
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A5
U 1 1 61D8E1CF
P 6750 1500
F 0 "A5" H 6600 1750 50  0000 L CNN
F 1 "DRV8871" H 6600 1250 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 6650 1500 50  0001 C CNN
F 3 "" H 6650 1500 50  0001 C CNN
	1    6750 1500
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A6
U 1 1 61D8E1C9
P 6750 2200
F 0 "A6" H 6600 2450 50  0000 L CNN
F 1 "DRV8871" H 6600 1950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 6650 2200 50  0001 C CNN
F 3 "" H 6650 2200 50  0001 C CNN
	1    6750 2200
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A7
U 1 1 61D8E1C3
P 6750 2900
F 0 "A7" H 6600 3150 50  0000 L CNN
F 1 "DRV8871" H 6600 2650 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 6650 2900 50  0001 C CNN
F 3 "" H 6650 2900 50  0001 C CNN
	1    6750 2900
	1    0    0    -1  
$EndComp
Text GLabel 5400 1450 0    50   Input ~ 0
AIN2
Text GLabel 5450 1350 0    50   Input ~ 0
AIN1
Wire Wire Line
	5300 2350 5300 3050
Connection ~ 5300 2350
Wire Wire Line
	5300 2350 5500 2350
Wire Wire Line
	5300 3050 5500 3050
Wire Wire Line
	5300 1650 5300 2350
Wire Wire Line
	5500 1650 5300 1650
Wire Wire Line
	5350 2250 5350 2950
Connection ~ 5350 2250
Wire Wire Line
	5350 2250 5500 2250
Wire Wire Line
	5350 2950 5500 2950
Wire Wire Line
	5350 1550 5350 2250
Wire Wire Line
	5500 1550 5350 1550
Connection ~ 5400 2150
Wire Wire Line
	5400 2850 5500 2850
Wire Wire Line
	5400 2150 5400 2850
Wire Wire Line
	5400 2150 5500 2150
Wire Wire Line
	5400 1450 5400 2150
Wire Wire Line
	5500 1450 5400 1450
Connection ~ 5450 2050
Wire Wire Line
	5450 2750 5500 2750
Wire Wire Line
	5450 2050 5450 2750
Wire Wire Line
	5450 2050 5500 2050
Wire Wire Line
	5450 1350 5450 2050
Wire Wire Line
	5500 1350 5450 1350
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A2
U 1 1 61D2F2F4
P 5750 1500
F 0 "A2" H 5600 1750 50  0000 L CNN
F 1 "DRV8871" H 5600 1250 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5650 1500 50  0001 C CNN
F 3 "" H 5650 1500 50  0001 C CNN
	1    5750 1500
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A3
U 1 1 61D314D6
P 5750 2200
F 0 "A3" H 5600 2450 50  0000 L CNN
F 1 "DRV8871" H 5600 1950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5650 2200 50  0001 C CNN
F 3 "" H 5650 2200 50  0001 C CNN
	1    5750 2200
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A4
U 1 1 61D32985
P 5750 2900
F 0 "A4" H 5600 3150 50  0000 L CNN
F 1 "DRV8871" H 5600 2650 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5650 2900 50  0001 C CNN
F 3 "" H 5650 2900 50  0001 C CNN
	1    5750 2900
	1    0    0    -1  
$EndComp
Text Notes 1000 2450 0    75   ~ 0
Microcontroller
$Comp
L power:GND #PWR02
U 1 1 61D84C64
P 1400 1700
F 0 "#PWR02" H 1400 1450 50  0001 C CNN
F 1 "GND" H 1405 1527 50  0000 C CNN
F 2 "" H 1400 1700 50  0001 C CNN
F 3 "" H 1400 1700 50  0001 C CNN
	1    1400 1700
	1    0    0    -1  
$EndComp
NoConn ~ 1400 3500
NoConn ~ 1400 3600
$Comp
L MCU_Module:Adafruit_Feather_328P A1
U 1 1 61D69E86
P 1900 3900
F 0 "A1" H 1550 5050 50  0000 C CNN
F 1 "Feather 328P" H 2150 2650 40  0000 C CNN
F 2 "Module:Adafruit_Feather" H 2000 2550 50  0001 L CNN
F 3 "https://cdn-learn.adafruit.com/downloads/pdf/adafruit-feather.pdf" H 1900 3100 50  0001 C CNN
	1    1900 3900
	1    0    0    -1  
$EndComp
Text GLabel 2400 3800 2    50   Input ~ 0
CHANNEL_1
Text GLabel 2400 3900 2    50   Input ~ 0
CHANNEL_2
Wire Notes Line
	1000 5500 3000 5500
NoConn ~ 1400 4100
Wire Notes Line
	3000 2500 3000 5500
Wire Notes Line
	1000 2500 3000 2500
Text GLabel 1400 3100 0    50   Input ~ 0
AIN2
Text GLabel 1400 3200 0    50   Input ~ 0
AIN1
Wire Notes Line
	3500 1000 1000 1000
Wire Notes Line
	1000 2000 3500 2000
Wire Wire Line
	1800 1300 1750 1300
$Comp
L power:VCC #PWR0102
U 1 1 61DE024E
P 1750 1300
F 0 "#PWR0102" H 1750 1150 50  0001 C CNN
F 1 "VCC" H 1765 1473 50  0000 C CNN
F 2 "" H 1750 1300 50  0001 C CNN
F 3 "" H 1750 1300 50  0001 C CNN
	1    1750 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 61DEEBD5
P 1950 1300
F 0 "D1" H 1950 1200 50  0000 C CNN
F 1 "D_Schottky" H 1950 1400 40  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-2_Vertical" H 1950 1300 50  0001 C CNN
F 3 "~" H 1950 1300 50  0001 C CNN
	1    1950 1300
	-1   0    0    1   
$EndComp
Connection ~ 2250 1300
Wire Wire Line
	2250 1300 2650 1300
Connection ~ 2650 1300
Wire Wire Line
	2650 1300 2750 1300
$Comp
L power:+12V #PWR0103
U 1 1 61DE130C
P 2650 1300
F 0 "#PWR0103" H 2650 1150 50  0001 C CNN
F 1 "+12V" H 2665 1473 50  0000 C CNN
F 2 "" H 2650 1300 50  0001 C CNN
F 3 "" H 2650 1300 50  0001 C CNN
	1    2650 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1300 3050 1300
$Comp
L Device:LED D3
U 1 1 61DE226D
P 3100 1450
F 0 "D3" V 3139 1332 50  0000 R CNN
F 1 "LED" V 3048 1332 50  0000 R CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 3100 1450 50  0001 C CNN
F 3 "~" H 3100 1450 50  0001 C CNN
	1    3100 1450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 61DE06EC
P 2900 1300
F 0 "R1" V 2709 1300 50  0000 C CNN
F 1 "698" V 2800 1300 40  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2830 1300 50  0001 C CNN
F 3 "~" H 2900 1300 50  0001 C CNN
	1    2900 1300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 61DDCF3D
P 2650 1700
F 0 "#PWR06" H 2650 1450 50  0001 C CNN
F 1 "GND" H 2655 1527 50  0000 C CNN
F 2 "" H 2650 1700 50  0001 C CNN
F 3 "" H 2650 1700 50  0001 C CNN
	1    2650 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 61DD9EA3
P 2250 1450
F 0 "C1" H 2368 1496 50  0000 L CNN
F 1 "2200uF" H 2350 1400 40  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 2288 1300 50  0001 C CNN
F 3 "~" H 2250 1450 50  0001 C CNN
	1    2250 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1300 2250 1300
Wire Notes Line
	3500 1000 3500 2000
$Comp
L power:+12V #PWR08
U 1 1 61E02A86
P 5350 1550
F 0 "#PWR08" H 5350 1400 50  0001 C CNN
F 1 "+12V" V 5365 1678 50  0000 L CNN
F 2 "" H 5350 1550 50  0001 C CNN
F 3 "" H 5350 1550 50  0001 C CNN
	1    5350 1550
	0    -1   -1   0   
$EndComp
Connection ~ 5350 1550
$Comp
L power:+12V #PWR09
U 1 1 61E02DBB
P 6350 1550
F 0 "#PWR09" H 6350 1400 50  0001 C CNN
F 1 "+12V" V 6365 1678 50  0000 L CNN
F 2 "" H 6350 1550 50  0001 C CNN
F 3 "" H 6350 1550 50  0001 C CNN
	1    6350 1550
	0    -1   -1   0   
$EndComp
Connection ~ 6350 1550
$Comp
L dk_Diodes-Zener-Single:1N5231BTR Z1
U 1 1 61DF337E
P 2650 1500
F 0 "Z1" V 2696 1422 50  0000 R CNN
F 1 "Diode" V 2605 1422 50  0000 R CNN
F 2 "digikey-footprints:DO-214AC" H 2850 1700 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/1N5221B-D.PDF" H 2850 1800 60  0001 L CNN
F 4 "1N5231BFSCT-ND" H 2850 1900 60  0001 L CNN "Digi-Key_PN"
F 5 "1N5231BTR" H 2850 2000 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 2850 2100 60  0001 L CNN "Category"
F 7 "Diodes - Zener - Single" H 2850 2200 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/1N5221B-D.PDF" H 2850 2300 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/1N5231BTR/1N5231BFSCT-ND/1532765" H 2850 2400 60  0001 L CNN "DK_Detail_Page"
F 10 "DIODE ZENER 5.1V 500MW DO35" H 2850 2500 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 2850 2600 60  0001 L CNN "Manufacturer"
F 12 "Active" H 2850 2700 60  0001 L CNN "Status"
	1    2650 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 1400 1400 1700
$Comp
L power:GND #PWR04
U 1 1 61E00E38
P 2250 1700
F 0 "#PWR04" H 2250 1450 50  0001 C CNN
F 1 "GND" H 2255 1527 50  0000 C CNN
F 2 "" H 2250 1700 50  0001 C CNN
F 3 "" H 2250 1700 50  0001 C CNN
	1    2250 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 61E018C8
P 3100 1700
F 0 "#PWR07" H 3100 1450 50  0001 C CNN
F 1 "GND" H 3105 1527 50  0000 C CNN
F 2 "" H 3100 1700 50  0001 C CNN
F 3 "" H 3100 1700 50  0001 C CNN
	1    3100 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1600 2250 1700
Wire Wire Line
	3100 1600 3100 1700
$EndSCHEMATC
