EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr User 9000 6500
encoding utf-8
Sheet 1 1
Title "All-Terrain Buggy "
Date "2022-01-11"
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
P 1100 1500
F 0 "J1" H 1018 1175 50  0000 C CNN
F 1 "XT60" H 1018 1266 50  0000 C CNN
F 2 "Connector_AMASS:AMASS_XT60-F_1x02_P7.20mm_Vertical" H 1100 1500 50  0001 C CNN
F 3 "~" H 1100 1500 50  0001 C CNN
	1    1100 1500
	-1   0    0    1   
$EndComp
$Comp
L power:VCC #PWR04
U 1 1 61DEDB89
P 2000 1400
F 0 "#PWR04" H 2000 1250 50  0001 C CNN
F 1 "VCC" H 2015 1573 50  0000 C CNN
F 2 "" H 2000 1400 50  0001 C CNN
F 3 "" H 2000 1400 50  0001 C CNN
	1    2000 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 61DEEBD5
P 2300 1400
F 0 "D1" H 2300 1183 50  0000 C CNN
F 1 "D_Schottky" H 2300 1274 40  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-2_Horizontal_TabDown" H 2300 1400 50  0001 C CNN
F 3 "~" H 2300 1400 50  0001 C CNN
	1    2300 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1400 1500 1400 1700
$Comp
L power:VCC #PWR01
U 1 1 61DF6D62
P 1400 1400
F 0 "#PWR01" H 1400 1250 50  0001 C CNN
F 1 "VCC" H 1415 1573 50  0000 C CNN
F 2 "" H 1400 1400 50  0001 C CNN
F 3 "" H 1400 1400 50  0001 C CNN
	1    1400 1400
	1    0    0    -1  
$EndComp
Wire Notes Line
	1000 1000 1000 2000
Wire Notes Line
	3500 1000 1000 1000
Wire Wire Line
	1300 1400 1400 1400
Wire Wire Line
	1300 1500 1400 1500
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
Text Notes 6000 950  0    75   ~ 0
Control Electronics
Wire Notes Line
	6000 1000 6000 3500
Wire Notes Line
	6000 3500 8000 3500
Wire Notes Line
	8000 1000 6000 1000
Wire Notes Line
	8000 3500 8000 1000
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
Text GLabel 1400 3200 0    50   Input ~ 0
AIN2
Text GLabel 1400 3100 0    50   Input ~ 0
AIN1
Connection ~ 7300 3050
Wire Wire Line
	7300 3050 7300 3200
Connection ~ 6300 3050
Wire Wire Line
	6300 3050 6300 3200
$Comp
L power:GND #PWR05
U 1 1 61D3B442
P 7300 3200
F 0 "#PWR05" H 7300 2950 50  0001 C CNN
F 1 "GND" H 7305 3027 50  0000 C CNN
F 2 "" H 7300 3200 50  0001 C CNN
F 3 "" H 7300 3200 50  0001 C CNN
	1    7300 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 61D3A90D
P 6300 3200
F 0 "#PWR03" H 6300 2950 50  0001 C CNN
F 1 "GND" H 6305 3027 50  0000 C CNN
F 2 "" H 6300 3200 50  0001 C CNN
F 3 "" H 6300 3200 50  0001 C CNN
	1    6300 3200
	1    0    0    -1  
$EndComp
Connection ~ 7350 1550
Wire Wire Line
	7100 1550 7100 1500
Wire Wire Line
	7350 1550 7100 1550
Text GLabel 7400 1450 0    50   Input ~ 0
BIN2
Text GLabel 7450 1350 0    50   Input ~ 0
BIN1
Wire Wire Line
	7300 2350 7300 3050
Connection ~ 7300 2350
Wire Wire Line
	7300 2350 7500 2350
Wire Wire Line
	7300 3050 7500 3050
Wire Wire Line
	7300 1650 7300 2350
Wire Wire Line
	7500 1650 7300 1650
Wire Wire Line
	7350 2250 7350 2950
Connection ~ 7350 2250
Wire Wire Line
	7350 2250 7500 2250
Wire Wire Line
	7350 2950 7500 2950
Wire Wire Line
	7350 1550 7350 2250
Wire Wire Line
	7500 1550 7350 1550
Connection ~ 7400 2150
Wire Wire Line
	7400 2850 7500 2850
Wire Wire Line
	7400 2150 7400 2850
Wire Wire Line
	7400 2150 7500 2150
Wire Wire Line
	7400 1450 7400 2150
Wire Wire Line
	7500 1450 7400 1450
Connection ~ 7450 2050
Wire Wire Line
	7450 2750 7500 2750
Wire Wire Line
	7450 2050 7450 2750
Wire Wire Line
	7450 2050 7500 2050
Wire Wire Line
	7450 1350 7450 2050
Wire Wire Line
	7500 1350 7450 1350
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A5
U 1 1 61D8E1CF
P 7750 1500
F 0 "A5" H 7600 1750 50  0000 L CNN
F 1 "DRV8871" H 7600 1250 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 7650 1500 50  0001 C CNN
F 3 "" H 7650 1500 50  0001 C CNN
	1    7750 1500
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A6
U 1 1 61D8E1C9
P 7750 2200
F 0 "A6" H 7600 2450 50  0000 L CNN
F 1 "DRV8871" H 7600 1950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 7650 2200 50  0001 C CNN
F 3 "" H 7650 2200 50  0001 C CNN
	1    7750 2200
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A7
U 1 1 61D8E1C3
P 7750 2900
F 0 "A7" H 7600 3150 50  0000 L CNN
F 1 "DRV8871" H 7600 2650 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 7650 2900 50  0001 C CNN
F 3 "" H 7650 2900 50  0001 C CNN
	1    7750 2900
	1    0    0    -1  
$EndComp
Connection ~ 6350 1550
Wire Wire Line
	6100 1550 6100 1500
Wire Wire Line
	6350 1550 6100 1550
Text GLabel 6400 1450 0    50   Input ~ 0
AIN2
Text GLabel 6450 1350 0    50   Input ~ 0
AIN1
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
L Buggy_Components:Adafruit_DRV8871_Breakout A2
U 1 1 61D2F2F4
P 6750 1500
F 0 "A2" H 6600 1750 50  0000 L CNN
F 1 "DRV8871" H 6600 1250 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 6650 1500 50  0001 C CNN
F 3 "" H 6650 1500 50  0001 C CNN
	1    6750 1500
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A3
U 1 1 61D314D6
P 6750 2200
F 0 "A3" H 6600 2450 50  0000 L CNN
F 1 "DRV8871" H 6600 1950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 6650 2200 50  0001 C CNN
F 3 "" H 6650 2200 50  0001 C CNN
	1    6750 2200
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A4
U 1 1 61D32985
P 6750 2900
F 0 "A4" H 6600 3150 50  0000 L CNN
F 1 "DRV8871" H 6600 2650 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 6650 2900 50  0001 C CNN
F 3 "" H 6650 2900 50  0001 C CNN
	1    6750 2900
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
Wire Notes Line
	3500 1000 3500 2000
Wire Notes Line
	3500 2000 1000 2000
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
Wire Wire Line
	2150 1400 2000 1400
$Comp
L power:+7.5V #PWR06
U 1 1 61DEFD0D
P 2600 1400
F 0 "#PWR06" H 2600 1250 50  0001 C CNN
F 1 "+7.5V" H 2615 1573 50  0000 C CNN
F 2 "" H 2600 1400 50  0001 C CNN
F 3 "" H 2600 1400 50  0001 C CNN
	1    2600 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 1400 2600 1400
$Comp
L power:+7.5V #PWR07
U 1 1 61DF115C
P 6100 1500
F 0 "#PWR07" H 6100 1350 50  0001 C CNN
F 1 "+7.5V" H 6115 1666 40  0000 C CNN
F 2 "" H 6100 1500 50  0001 C CNN
F 3 "" H 6100 1500 50  0001 C CNN
	1    6100 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+7.5V #PWR08
U 1 1 61DF13CD
P 7100 1500
F 0 "#PWR08" H 7100 1350 50  0001 C CNN
F 1 "+7.5V" H 7115 1666 40  0000 C CNN
F 2 "" H 7100 1500 50  0001 C CNN
F 3 "" H 7100 1500 50  0001 C CNN
	1    7100 1500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
