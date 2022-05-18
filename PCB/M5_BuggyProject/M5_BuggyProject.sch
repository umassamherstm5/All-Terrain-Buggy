EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "All-Terrain Buggy "
Date "2022-05-10"
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
Text Notes 8750 950  0    75   ~ 0
Control Electronics / Drivetrain
Wire Notes Line
	8750 1000 8750 3500
Wire Notes Line
	8750 3500 10750 3500
Wire Notes Line
	10750 1000 8750 1000
Wire Notes Line
	10750 3500 10750 1000
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
Wire Wire Line
	2200 1300 2150 1300
$Comp
L power:VCC #PWR0102
U 1 1 61DE024E
P 2150 1300
F 0 "#PWR0102" H 2150 1150 50  0001 C CNN
F 1 "VCC" H 2165 1473 50  0000 C CNN
F 2 "" H 2150 1300 50  0001 C CNN
F 3 "" H 2150 1300 50  0001 C CNN
	1    2150 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 61DEEBD5
P 2350 1300
F 0 "D1" H 2350 1200 50  0000 C CNN
F 1 "D_Schottky" H 2350 1400 40  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-2_Vertical" H 2350 1300 50  0001 C CNN
F 3 "~" H 2350 1300 50  0001 C CNN
	1    2350 1300
	-1   0    0    1   
$EndComp
Connection ~ 2650 1300
Wire Wire Line
	2650 1300 3050 1300
Connection ~ 3050 1300
Wire Wire Line
	3050 1300 3150 1300
Wire Wire Line
	3500 1300 3450 1300
$Comp
L Device:LED D3
U 1 1 61DE226D
P 3500 1450
F 0 "D3" V 3539 1332 50  0000 R CNN
F 1 "BLUE" V 3448 1332 50  0000 R CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 3500 1450 50  0001 C CNN
F 3 "~" H 3500 1450 50  0001 C CNN
	1    3500 1450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 61DE06EC
P 3300 1300
F 0 "R1" V 3109 1300 50  0000 C CNN
F 1 "330" V 3200 1300 40  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3230 1300 50  0001 C CNN
F 3 "~" H 3300 1300 50  0001 C CNN
	1    3300 1300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 61DDCF3D
P 3050 1700
F 0 "#PWR06" H 3050 1450 50  0001 C CNN
F 1 "GND" H 3055 1527 50  0000 C CNN
F 2 "" H 3050 1700 50  0001 C CNN
F 3 "" H 3050 1700 50  0001 C CNN
	1    3050 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 61DD9EA3
P 2650 1450
F 0 "C1" H 2768 1496 50  0000 L CNN
F 1 "2200uF" H 2750 1400 40  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 2688 1300 50  0001 C CNN
F 3 "~" H 2650 1450 50  0001 C CNN
	1    2650 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1300 2650 1300
$Comp
L M5_BuggyProject-rescue:1N5231BTR-dk_Diodes-Zener-Single Z1
U 1 1 61DF337E
P 3050 1500
F 0 "Z1" V 3096 1422 50  0000 R CNN
F 1 "Zener" V 3005 1422 50  0000 R CNN
F 2 "digikey-footprints:DO-214AC" H 3250 1700 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/1N5221B-D.PDF" H 3250 1800 60  0001 L CNN
F 4 "1N5231BFSCT-ND" H 3250 1900 60  0001 L CNN "Digi-Key_PN"
F 5 "1N5231BTR" H 3250 2000 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 3250 2100 60  0001 L CNN "Category"
F 7 "Diodes - Zener - Single" H 3250 2200 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/1N5221B-D.PDF" H 3250 2300 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/1N5231BTR/1N5231BFSCT-ND/1532765" H 3250 2400 60  0001 L CNN "DK_Detail_Page"
F 10 "DIODE ZENER 5.1V 500MW DO35" H 3250 2500 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 3250 2600 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3250 2700 60  0001 L CNN "Status"
	1    3050 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 1400 1400 1700
$Comp
L power:GND #PWR04
U 1 1 61E00E38
P 2650 1700
F 0 "#PWR04" H 2650 1450 50  0001 C CNN
F 1 "GND" H 2655 1527 50  0000 C CNN
F 2 "" H 2650 1700 50  0001 C CNN
F 3 "" H 2650 1700 50  0001 C CNN
	1    2650 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 61E018C8
P 3500 1700
F 0 "#PWR07" H 3500 1450 50  0001 C CNN
F 1 "GND" H 3505 1527 50  0000 C CNN
F 2 "" H 3500 1700 50  0001 C CNN
F 3 "" H 3500 1700 50  0001 C CNN
	1    3500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1600 2650 1700
Wire Wire Line
	3500 1600 3500 1700
Wire Notes Line
	5650 1000 6650 1000
Text Notes 5650 950  0    75   ~ 0
Mounting Holes
Wire Notes Line
	1000 1000 4000 1000
Wire Notes Line
	1000 2000 4000 2000
Wire Notes Line
	4000 2000 4000 1000
$Comp
L power:+10V #PWR08
U 1 1 61E1C277
P 3050 1300
F 0 "#PWR08" H 3050 1150 50  0001 C CNN
F 1 "+10V" H 3065 1473 50  0000 C CNN
F 2 "" H 3050 1300 50  0001 C CNN
F 3 "" H 3050 1300 50  0001 C CNN
	1    3050 1300
	1    0    0    -1  
$EndComp
Wire Notes Line
	6650 2000 5650 2000
Wire Notes Line
	6650 1000 6650 2000
Wire Notes Line
	5650 1000 5650 2000
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 61E14C56
P 5850 1200
F 0 "H1" H 5950 1249 50  0000 L CNN
F 1 "M" H 5950 1158 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 5850 1200 50  0001 C CNN
F 3 "~" H 5850 1200 50  0001 C CNN
	1    5850 1200
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 61E16060
P 5850 1700
F 0 "H2" H 5950 1749 50  0000 L CNN
F 1 "M" H 5950 1658 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 5850 1700 50  0001 C CNN
F 3 "~" H 5850 1700 50  0001 C CNN
	1    5850 1700
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 61E16E7D
P 6350 1700
F 0 "H4" H 6450 1749 50  0000 L CNN
F 1 "M" H 6450 1658 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 6350 1700 50  0001 C CNN
F 3 "~" H 6350 1700 50  0001 C CNN
	1    6350 1700
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 61E15C74
P 6350 1200
F 0 "H3" H 6450 1249 50  0000 L CNN
F 1 "M" H 6450 1158 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 6350 1200 50  0001 C CNN
F 3 "~" H 6350 1200 50  0001 C CNN
	1    6350 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 61E86183
P 6900 4400
F 0 "#PWR019" H 6900 4150 50  0001 C CNN
F 1 "GND" H 6905 4227 50  0000 C CNN
F 2 "" H 6900 4400 50  0001 C CNN
F 3 "" H 6900 4400 50  0001 C CNN
	1    6900 4400
	1    0    0    -1  
$EndComp
Text GLabel 6900 3800 0    50   Input ~ 0
BEEP
Wire Wire Line
	6900 4000 6900 4400
Connection ~ 10100 1550
$Comp
L power:+10V #PWR016
U 1 1 61E1D60D
P 10100 1550
F 0 "#PWR016" H 10100 1400 50  0001 C CNN
F 1 "+10V" V 10115 1678 50  0000 L CNN
F 2 "" H 10100 1550 50  0001 C CNN
F 3 "" H 10100 1550 50  0001 C CNN
	1    10100 1550
	0    -1   -1   0   
$EndComp
Connection ~ 9100 1550
$Comp
L power:+10V #PWR011
U 1 1 61E1D1B6
P 9100 1550
F 0 "#PWR011" H 9100 1400 50  0001 C CNN
F 1 "+10V" V 9115 1678 50  0000 L CNN
F 2 "" H 9100 1550 50  0001 C CNN
F 3 "" H 9100 1550 50  0001 C CNN
	1    9100 1550
	0    -1   -1   0   
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A4
U 1 1 61D32985
P 9500 2900
F 0 "A4" H 9350 3150 50  0000 L CNN
F 1 "DRV8871" H 9350 2650 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 9400 2900 50  0001 C CNN
F 3 "" H 9400 2900 50  0001 C CNN
	1    9500 2900
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A3
U 1 1 61D314D6
P 9500 2200
F 0 "A3" H 9350 2450 50  0000 L CNN
F 1 "DRV8871" H 9350 1950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 9400 2200 50  0001 C CNN
F 3 "" H 9400 2200 50  0001 C CNN
	1    9500 2200
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A2
U 1 1 61D2F2F4
P 9500 1500
F 0 "A2" H 9350 1750 50  0000 L CNN
F 1 "DRV8871" H 9350 1250 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 9400 1500 50  0001 C CNN
F 3 "" H 9400 1500 50  0001 C CNN
	1    9500 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 1350 9200 1350
Wire Wire Line
	9200 1350 9200 2050
Wire Wire Line
	9200 2050 9250 2050
Wire Wire Line
	9200 2050 9200 2750
Wire Wire Line
	9200 2750 9250 2750
Connection ~ 9200 2050
Wire Wire Line
	9250 1450 9150 1450
Wire Wire Line
	9150 1450 9150 2150
Wire Wire Line
	9150 2150 9250 2150
Wire Wire Line
	9150 2150 9150 2850
Wire Wire Line
	9150 2850 9250 2850
Connection ~ 9150 2150
Wire Wire Line
	9250 1550 9100 1550
Wire Wire Line
	9100 1550 9100 2250
Wire Wire Line
	9100 2950 9250 2950
Wire Wire Line
	9100 2250 9250 2250
Connection ~ 9100 2250
Wire Wire Line
	9100 2250 9100 2950
Wire Wire Line
	9250 1650 9050 1650
Wire Wire Line
	9050 1650 9050 2350
Wire Wire Line
	9050 3050 9250 3050
Wire Wire Line
	9050 2350 9250 2350
Connection ~ 9050 2350
Wire Wire Line
	9050 2350 9050 3050
Text GLabel 9200 1350 0    50   Input ~ 0
AIN1
Text GLabel 9150 1450 0    50   Input ~ 0
AIN2
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A7
U 1 1 61D8E1C3
P 10500 2900
F 0 "A7" H 10350 3150 50  0000 L CNN
F 1 "DRV8871" H 10350 2650 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 10400 2900 50  0001 C CNN
F 3 "" H 10400 2900 50  0001 C CNN
	1    10500 2900
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A6
U 1 1 61D8E1C9
P 10500 2200
F 0 "A6" H 10350 2450 50  0000 L CNN
F 1 "DRV8871" H 10350 1950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 10400 2200 50  0001 C CNN
F 3 "" H 10400 2200 50  0001 C CNN
	1    10500 2200
	1    0    0    -1  
$EndComp
$Comp
L Buggy_Components:Adafruit_DRV8871_Breakout A5
U 1 1 61D8E1CF
P 10500 1500
F 0 "A5" H 10350 1750 50  0000 L CNN
F 1 "DRV8871" H 10350 1250 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 10400 1500 50  0001 C CNN
F 3 "" H 10400 1500 50  0001 C CNN
	1    10500 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 1350 10200 1350
Wire Wire Line
	10200 1350 10200 2050
Wire Wire Line
	10200 2050 10250 2050
Wire Wire Line
	10200 2050 10200 2750
Wire Wire Line
	10200 2750 10250 2750
Connection ~ 10200 2050
Wire Wire Line
	10250 1450 10150 1450
Wire Wire Line
	10150 1450 10150 2150
Wire Wire Line
	10150 2150 10250 2150
Wire Wire Line
	10150 2150 10150 2850
Wire Wire Line
	10150 2850 10250 2850
Connection ~ 10150 2150
Wire Wire Line
	10250 1550 10100 1550
Wire Wire Line
	10100 1550 10100 2250
Wire Wire Line
	10100 2950 10250 2950
Wire Wire Line
	10100 2250 10250 2250
Connection ~ 10100 2250
Wire Wire Line
	10100 2250 10100 2950
Wire Wire Line
	10250 1650 10050 1650
Wire Wire Line
	10050 1650 10050 2350
Wire Wire Line
	10050 3050 10250 3050
Wire Wire Line
	10050 2350 10250 2350
Connection ~ 10050 2350
Wire Wire Line
	10050 2350 10050 3050
Text GLabel 10200 1350 0    50   Input ~ 0
BIN1
Text GLabel 10150 1450 0    50   Input ~ 0
BIN2
$Comp
L power:GND #PWR03
U 1 1 61D3A90D
P 9050 3200
F 0 "#PWR03" H 9050 2950 50  0001 C CNN
F 1 "GND" H 9055 3027 50  0000 C CNN
F 2 "" H 9050 3200 50  0001 C CNN
F 3 "" H 9050 3200 50  0001 C CNN
	1    9050 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 61D3B442
P 10050 3200
F 0 "#PWR05" H 10050 2950 50  0001 C CNN
F 1 "GND" H 10055 3027 50  0000 C CNN
F 2 "" H 10050 3200 50  0001 C CNN
F 3 "" H 10050 3200 50  0001 C CNN
	1    10050 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 3050 9050 3200
Connection ~ 9050 3050
Wire Wire Line
	10050 3050 10050 3200
Connection ~ 10050 3050
$Comp
L Device:Buzzer BZ1
U 1 1 61E818D3
P 7000 3900
F 0 "BZ1" H 7152 3929 50  0000 L CNN
F 1 "BEEP" H 7152 3838 50  0000 L CNN
F 2 "Buzzer_Beeper:Buzzer_12x9.5RM7.6" V 6975 4000 50  0001 C CNN
F 3 "~" V 6975 4000 50  0001 C CNN
	1    7000 3900
	1    0    0    -1  
$EndComp
Wire Notes Line
	4450 3500 4450 5500
$Comp
L Connector_Generic:Conn_01x05 J4
U 1 1 6201B3C5
P 5300 5200
F 0 "J4" V 5172 5480 50  0000 L CNN
F 1 "Receiver" V 5263 5480 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 5300 5200 50  0001 C CNN
F 3 "~" H 5300 5200 50  0001 C CNN
	1    5300 5200
	0    1    1    0   
$EndComp
Text GLabel 5100 5000 0    50   Input ~ 0
CHANNEL_1
Wire Wire Line
	5200 5000 5200 4900
Wire Wire Line
	5200 4900 5100 4900
Text GLabel 5100 4900 0    50   Input ~ 0
CHANNEL_2
Wire Wire Line
	5300 5000 5300 4800
Wire Wire Line
	5300 4800 5100 4800
Text GLabel 5100 4800 0    50   Input ~ 0
CHANNEL_3
$Comp
L power:+3.3V #PWR022
U 1 1 62022951
P 5400 5000
F 0 "#PWR022" H 5400 4850 50  0001 C CNN
F 1 "+3.3V" H 5415 5173 50  0000 C CNN
F 2 "" H 5400 5000 50  0001 C CNN
F 3 "" H 5400 5000 50  0001 C CNN
	1    5400 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 62022B50
P 5500 5000
F 0 "#PWR023" H 5500 4750 50  0001 C CNN
F 1 "GND" V 5505 4872 50  0000 R CNN
F 2 "" H 5500 5000 50  0001 C CNN
F 3 "" H 5500 5000 50  0001 C CNN
	1    5500 5000
	0    -1   -1   0   
$EndComp
Wire Notes Line
	7750 5500 7750 3500
Wire Notes Line
	4450 5500 7750 5500
NoConn ~ 5850 1300
NoConn ~ 6350 1300
NoConn ~ 6350 1800
NoConn ~ 5850 1800
Text Notes 4450 3450 0    75   ~ 0
MCU Connections + Testpoints
NoConn ~ 2450 5250
NoConn ~ 2450 5150
$Comp
L power:+5V #PWR021
U 1 1 61F86A6B
P 2150 3750
F 0 "#PWR021" H 2150 3600 50  0001 C CNN
F 1 "+5V" V 2165 3878 50  0000 L CNN
F 2 "" H 2150 3750 50  0001 C CNN
F 3 "" H 2150 3750 50  0001 C CNN
	1    2150 3750
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR020
U 1 1 61F83393
P 2050 3750
F 0 "#PWR020" H 2050 3600 50  0001 C CNN
F 1 "+3.3V" H 2065 3923 50  0000 C CNN
F 2 "" H 2050 3750 50  0001 C CNN
F 3 "" H 2050 3750 50  0001 C CNN
	1    2050 3750
	1    0    0    -1  
$EndComp
Text GLabel 2450 5050 2    50   Input ~ 0
CHANNEL_3
Text GLabel 1450 4650 0    50   Input ~ 0
BEEP
NoConn ~ 2450 4750
NoConn ~ 1450 5250
NoConn ~ 1450 5050
NoConn ~ 1450 4850
NoConn ~ 1450 4750
NoConn ~ 1450 5850
NoConn ~ 1450 5750
NoConn ~ 1450 5550
NoConn ~ 1450 5450
NoConn ~ 2450 4550
NoConn ~ 2450 4450
NoConn ~ 2450 4150
NoConn ~ 1850 3750
$Comp
L power:GND #PWR0101
U 1 1 61D8BB98
P 1950 6250
F 0 "#PWR0101" H 1950 6000 50  0001 C CNN
F 1 "GND" H 1955 6077 50  0000 C CNN
F 2 "" H 1950 6250 50  0001 C CNN
F 3 "" H 1950 6250 50  0001 C CNN
	1    1950 6250
	1    0    0    -1  
$EndComp
Text GLabel 1450 4450 0    50   Input ~ 0
BIN2
Text GLabel 1450 4350 0    50   Input ~ 0
BIN1
NoConn ~ 1450 4550
$Comp
L M5_BuggyProject-rescue:Adafruit_Feather_328P-MCU_Module A1
U 1 1 61D69E86
P 1950 4950
F 0 "A1" H 1600 6100 50  0000 C CNN
F 1 "Feather 328P" H 2200 3700 40  0000 C CNN
F 2 "Module:Adafruit_Feather" H 2050 3600 50  0001 L CNN
F 3 "https://cdn-learn.adafruit.com/downloads/pdf/adafruit-feather.pdf" H 1950 4150 50  0001 C CNN
	1    1950 4950
	1    0    0    -1  
$EndComp
Text GLabel 2450 4850 2    50   Input ~ 0
CHANNEL_1
Text GLabel 2450 4950 2    50   Input ~ 0
CHANNEL_2
Wire Notes Line
	1000 6500 3000 6500
NoConn ~ 1450 5150
Text GLabel 1450 4150 0    50   Input ~ 0
AIN2
Text GLabel 1450 4250 0    50   Input ~ 0
AIN1
Wire Notes Line
	3000 3500 3000 6500
Text Notes 1000 3450 0    75   ~ 0
Microcontroller
Wire Notes Line
	1000 6500 1000 3500
Wire Notes Line
	1000 3500 3000 3500
Wire Notes Line
	4450 3500 7750 3500
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 627EA30D
P 4700 4200
F 0 "J2" V 4572 4280 50  0000 L CNN
F 1 "VCC" V 4663 4280 50  0000 L CNN
F 2 "TestPoint:TestPoint_Plated_Hole_D5.0mm" H 4700 4200 50  0001 C CNN
F 3 "~" H 4700 4200 50  0001 C CNN
	1    4700 4200
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 627EAC87
P 5200 4200
F 0 "J3" V 5072 4280 50  0000 L CNN
F 1 "10V" V 5163 4280 50  0000 L CNN
F 2 "TestPoint:TestPoint_Plated_Hole_D5.0mm" H 5200 4200 50  0001 C CNN
F 3 "~" H 5200 4200 50  0001 C CNN
	1    5200 4200
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 627EB291
P 5700 4200
F 0 "J5" V 5572 4280 50  0000 L CNN
F 1 "GND" V 5663 4280 50  0000 L CNN
F 2 "TestPoint:TestPoint_Plated_Hole_D5.0mm" H 5700 4200 50  0001 C CNN
F 3 "~" H 5700 4200 50  0001 C CNN
	1    5700 4200
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J9
U 1 1 627EBE1A
P 7500 5150
F 0 "J9" V 7500 5050 50  0000 L CNN
F 1 "BIN2" V 7600 5050 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 7500 5150 50  0001 C CNN
F 3 "~" H 7500 5150 50  0001 C CNN
	1    7500 5150
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J8
U 1 1 627ED52B
P 7100 5150
F 0 "J8" V 7100 5050 50  0000 L CNN
F 1 "BIN1" V 7200 5050 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 7100 5150 50  0001 C CNN
F 3 "~" H 7100 5150 50  0001 C CNN
	1    7100 5150
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 627EDB03
P 6700 5150
F 0 "J7" V 6700 5050 50  0000 L CNN
F 1 "AIN2" V 6800 5050 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 6700 5150 50  0001 C CNN
F 3 "~" H 6700 5150 50  0001 C CNN
	1    6700 5150
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J6
U 1 1 627EE707
P 6300 5150
F 0 "J6" V 6300 5050 50  0000 L CNN
F 1 "AIN1" V 6400 5050 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 6300 5150 50  0001 C CNN
F 3 "~" H 6300 5150 50  0001 C CNN
	1    6300 5150
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0103
U 1 1 627F6771
P 4700 4000
F 0 "#PWR0103" H 4700 3850 50  0001 C CNN
F 1 "VCC" H 4715 4173 50  0000 C CNN
F 2 "" H 4700 4000 50  0001 C CNN
F 3 "" H 4700 4000 50  0001 C CNN
	1    4700 4000
	1    0    0    -1  
$EndComp
$Comp
L power:+10V #PWR0104
U 1 1 627F6F82
P 5200 4000
F 0 "#PWR0104" H 5200 3850 50  0001 C CNN
F 1 "+10V" H 5215 4173 50  0000 C CNN
F 2 "" H 5200 4000 50  0001 C CNN
F 3 "" H 5200 4000 50  0001 C CNN
	1    5200 4000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 627F793F
P 5700 4000
F 0 "#PWR0105" H 5700 3750 50  0001 C CNN
F 1 "GND" H 5705 3827 50  0000 C CNN
F 2 "" H 5700 4000 50  0001 C CNN
F 3 "" H 5700 4000 50  0001 C CNN
	1    5700 4000
	-1   0    0    1   
$EndComp
Text GLabel 6200 4950 0    50   Input ~ 0
AIN1
Text GLabel 6600 4950 0    50   Input ~ 0
AIN2
Text GLabel 7000 4950 0    50   Input ~ 0
BIN1
Text GLabel 7400 4950 0    50   Input ~ 0
BIN2
$Comp
L power:GND #PWR0106
U 1 1 627FA39F
P 6300 4800
F 0 "#PWR0106" H 6300 4550 50  0001 C CNN
F 1 "GND" H 6305 4627 50  0000 C CNN
F 2 "" H 6300 4800 50  0001 C CNN
F 3 "" H 6300 4800 50  0001 C CNN
	1    6300 4800
	-1   0    0    1   
$EndComp
Wire Wire Line
	6300 4950 6300 4800
Wire Wire Line
	6300 4800 6700 4800
Wire Wire Line
	6700 4800 6700 4950
Connection ~ 6300 4800
Wire Wire Line
	6700 4800 7100 4800
Wire Wire Line
	7100 4800 7100 4950
Connection ~ 6700 4800
Wire Wire Line
	7100 4800 7500 4800
Wire Wire Line
	7500 4800 7500 4950
Connection ~ 7100 4800
$EndSCHEMATC
