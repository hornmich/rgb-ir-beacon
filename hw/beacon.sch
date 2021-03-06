EESchema Schematic File Version 4
LIBS:beacon-cache
EELAYER 26 0
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
L MCU_Microchip_ATmega:ATmega328-AU U1
U 1 1 5B9B91BF
P 4750 3500
F 0 "U1" H 4300 5000 50  0000 C CNN
F 1 "ATmega328-AU" H 5200 5000 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 4750 3500 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 4750 3500 50  0001 C CNN
	1    4750 3500
	1    0    0    -1  
$EndComp
$Comp
L Interface_Optical:TSOP382xx U2
U 1 1 5B9B9759
P 8450 1650
F 0 "U2" H 8437 2075 50  0000 C CNN
F 1 "TSOP382xx" H 8437 1984 50  0000 C CNN
F 2 "myFootprints:Vishay_MINIMOLD-3Pin-horizontal-smd" H 8400 1275 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82491/tsop382.pdf" H 9100 1950 50  0001 C CNN
	1    8450 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_ARGB D1
U 1 1 5B9B9800
P 8950 3250
F 0 "D1" H 8950 3747 50  0000 C CNN
F 1 "LED_ARGB" H 8950 3656 50  0000 C CNN
F 2 "myFootprints:LED_RGB_3W" H 8950 3200 50  0001 C CNN
F 3 "~" H 8950 3200 50  0001 C CNN
	1    8950 3250
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R5
U 1 1 5B9B9871
P 8600 3900
F 0 "R5" H 8530 3854 50  0000 R CNN
F 1 "220R" H 8530 3945 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8530 3900 50  0001 C CNN
F 3 "~" H 8600 3900 50  0001 C CNN
	1    8600 3900
	-1   0    0    1   
$EndComp
$Comp
L Device:R R7
U 1 1 5B9B98ED
P 8950 3900
F 0 "R7" H 9020 3946 50  0000 L CNN
F 1 "220R" H 9020 3855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8880 3900 50  0001 C CNN
F 3 "~" H 8950 3900 50  0001 C CNN
	1    8950 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5B9B990F
P 9300 3900
F 0 "R9" H 9370 3946 50  0000 L CNN
F 1 "220R" H 9370 3855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9230 3900 50  0001 C CNN
F 3 "~" H 9300 3900 50  0001 C CNN
	1    9300 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5B9B998E
P 4550 1700
F 0 "C2" V 4700 1700 50  0000 C CNN
F 1 "10n" V 4650 1600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4588 1550 50  0001 C CNN
F 3 "~" H 4550 1700 50  0001 C CNN
	1    4550 1700
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0101
U 1 1 5B9B9B69
P 4750 1100
F 0 "#PWR0101" H 4750 950 50  0001 C CNN
F 1 "VCC" H 4767 1273 50  0000 C CNN
F 2 "" H 4750 1100 50  0001 C CNN
F 3 "" H 4750 1100 50  0001 C CNN
	1    4750 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5B9B9BB4
P 4750 5250
F 0 "#PWR0102" H 4750 5000 50  0001 C CNN
F 1 "GND" H 4755 5077 50  0000 C CNN
F 2 "" H 4750 5250 50  0001 C CNN
F 3 "" H 4750 5250 50  0001 C CNN
	1    4750 5250
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0103
U 1 1 5B9BA083
P 8950 2900
F 0 "#PWR0103" H 8950 2750 50  0001 C CNN
F 1 "VCC" V 8967 3028 50  0000 L CNN
F 2 "" H 8950 2900 50  0001 C CNN
F 3 "" H 8950 2900 50  0001 C CNN
	1    8950 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5B9BA3BF
P 4550 1350
F 0 "C1" V 4700 1350 50  0000 C CNN
F 1 "10n" V 4600 1250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4588 1200 50  0001 C CNN
F 3 "~" H 4550 1350 50  0001 C CNN
	1    4550 1350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5B9BA5F0
P 4000 1750
F 0 "#PWR0104" H 4000 1500 50  0001 C CNN
F 1 "GND" H 4005 1577 50  0000 C CNN
F 2 "" H 4000 1750 50  0001 C CNN
F 3 "" H 4000 1750 50  0001 C CNN
	1    4000 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 2000 4750 1700
Wire Wire Line
	4700 1350 4750 1350
Connection ~ 4750 1350
Wire Wire Line
	4750 1350 4750 1100
Connection ~ 4750 1700
Wire Wire Line
	4750 1700 4750 1350
Wire Wire Line
	4300 1550 4000 1550
Wire Wire Line
	4000 1550 4000 1750
Wire Wire Line
	4300 1550 4300 1700
Wire Wire Line
	4700 1700 4750 1700
Wire Wire Line
	4300 1700 4400 1700
Wire Wire Line
	4400 1350 4300 1350
Wire Wire Line
	4300 1350 4300 1550
Connection ~ 4300 1550
Wire Wire Line
	4750 5000 4750 5250
Wire Wire Line
	8600 3750 8600 3550
Wire Wire Line
	8600 3550 8750 3550
Wire Wire Line
	8750 3550 8750 3450
Wire Wire Line
	8950 3750 8950 3450
Wire Wire Line
	9300 3750 9300 3550
Wire Wire Line
	9300 3550 9150 3550
Wire Wire Line
	9150 3550 9150 3450
$Comp
L power:GND #PWR0105
U 1 1 5B9BB00B
P 9000 2000
F 0 "#PWR0105" H 9000 1750 50  0001 C CNN
F 1 "GND" H 9005 1827 50  0000 C CNN
F 2 "" H 9000 2000 50  0001 C CNN
F 3 "" H 9000 2000 50  0001 C CNN
	1    9000 2000
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0106
U 1 1 5B9BB08F
P 9000 1300
F 0 "#PWR0106" H 9000 1150 50  0001 C CNN
F 1 "VCC" H 9017 1473 50  0000 C CNN
F 2 "" H 9000 1300 50  0001 C CNN
F 3 "" H 9000 1300 50  0001 C CNN
	1    9000 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 1450 9000 1450
Wire Wire Line
	9000 1450 9000 1300
Wire Wire Line
	8850 1850 9000 1850
Wire Wire Line
	9000 1850 9000 2000
$Comp
L Device:R R8
U 1 1 5B9BB40C
P 9150 1650
F 0 "R8" V 9357 1650 50  0000 C CNN
F 1 "100R" V 9266 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9080 1650 50  0001 C CNN
F 3 "~" H 9150 1650 50  0001 C CNN
	1    9150 1650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8850 1650 9000 1650
Wire Wire Line
	9300 1650 9550 1650
Text Label 9550 1650 0    50   ~ 0
IR_MCU_IN
$Comp
L power:GND #PWR0107
U 1 1 5B9BBE22
P 7650 5300
F 0 "#PWR0107" H 7650 5050 50  0001 C CNN
F 1 "GND" H 7655 5127 50  0000 C CNN
F 2 "" H 7650 5300 50  0001 C CNN
F 3 "" H 7650 5300 50  0001 C CNN
	1    7650 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3050 8950 2900
$Comp
L Device:R R2
U 1 1 5B9BC106
P 7100 4750
F 0 "R2" V 7307 4750 50  0000 C CNN
F 1 "100R" V 7216 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7030 4750 50  0001 C CNN
F 3 "~" H 7100 4750 50  0001 C CNN
	1    7100 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7250 4750 7300 4750
$Comp
L Device:R R3
U 1 1 5B9BC47C
P 7300 5000
F 0 "R3" H 7370 5046 50  0000 L CNN
F 1 "10k" H 7370 4955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7230 5000 50  0001 C CNN
F 3 "~" H 7300 5000 50  0001 C CNN
	1    7300 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 4850 7300 4750
Connection ~ 7300 4750
Wire Wire Line
	7300 4750 7350 4750
Wire Wire Line
	7300 5150 7650 5150
Wire Wire Line
	7650 5150 7650 5300
Wire Wire Line
	7650 5150 7650 4950
Connection ~ 7650 5150
Wire Wire Line
	6950 4750 6700 4750
Text Label 6700 4750 0    50   ~ 0
LED_R_MCU_OUT
$Comp
L power:GND #PWR0108
U 1 1 5B9BE98E
P 8950 5300
F 0 "#PWR0108" H 8950 5050 50  0001 C CNN
F 1 "GND" H 8955 5127 50  0000 C CNN
F 2 "" H 8950 5300 50  0001 C CNN
F 3 "" H 8950 5300 50  0001 C CNN
	1    8950 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5B9BE994
P 8400 4750
F 0 "R4" V 8607 4750 50  0000 C CNN
F 1 "100R" V 8516 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8330 4750 50  0001 C CNN
F 3 "~" H 8400 4750 50  0001 C CNN
	1    8400 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8550 4750 8600 4750
$Comp
L Device:R R6
U 1 1 5B9BE99B
P 8600 5000
F 0 "R6" H 8670 5046 50  0000 L CNN
F 1 "10k" H 8670 4955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8530 5000 50  0001 C CNN
F 3 "~" H 8600 5000 50  0001 C CNN
	1    8600 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 4850 8600 4750
Connection ~ 8600 4750
Wire Wire Line
	8600 4750 8650 4750
Wire Wire Line
	8600 5150 8950 5150
Wire Wire Line
	8950 5150 8950 5300
Wire Wire Line
	8950 5150 8950 4950
Connection ~ 8950 5150
Wire Wire Line
	8250 4750 8000 4750
Text Label 8000 4750 0    50   ~ 0
LED_G_MCU_OUT
$Comp
L power:GND #PWR0109
U 1 1 5B9BF057
P 10300 5300
F 0 "#PWR0109" H 10300 5050 50  0001 C CNN
F 1 "GND" H 10305 5127 50  0000 C CNN
F 2 "" H 10300 5300 50  0001 C CNN
F 3 "" H 10300 5300 50  0001 C CNN
	1    10300 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5B9BF05D
P 9750 4750
F 0 "R10" V 9957 4750 50  0000 C CNN
F 1 "100R" V 9866 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9680 4750 50  0001 C CNN
F 3 "~" H 9750 4750 50  0001 C CNN
	1    9750 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9900 4750 9950 4750
$Comp
L Device:R R11
U 1 1 5B9BF064
P 9950 5000
F 0 "R11" H 10020 5046 50  0000 L CNN
F 1 "10k" H 10020 4955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9880 5000 50  0001 C CNN
F 3 "~" H 9950 5000 50  0001 C CNN
	1    9950 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 4850 9950 4750
Connection ~ 9950 4750
Wire Wire Line
	9950 4750 10000 4750
Wire Wire Line
	9950 5150 10300 5150
Wire Wire Line
	10300 5150 10300 5300
Wire Wire Line
	10300 5150 10300 4950
Connection ~ 10300 5150
Wire Wire Line
	9600 4750 9350 4750
Text Label 9350 4750 0    50   ~ 0
LED_B_MCU_OUT
Wire Wire Line
	7650 4550 7650 4050
Wire Wire Line
	7650 4050 8600 4050
Wire Wire Line
	8950 4050 8950 4550
Wire Wire Line
	9300 4050 10300 4050
Wire Wire Line
	10300 4050 10300 4550
$Comp
L Connector_Generic:Conn_02x03_Top_Bottom J1
U 1 1 5B9C48BA
P 6550 1250
F 0 "J1" H 6600 1567 50  0000 C CNN
F 1 "SPI_Prog_conn" H 6600 1476 50  0000 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x03_P1.27mm_Vertical_SMD" H 6550 1250 50  0001 C CNN
F 3 "~" H 6550 1250 50  0001 C CNN
	1    6550 1250
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0110
U 1 1 5B9C49CE
P 7250 1150
F 0 "#PWR0110" H 7250 1000 50  0001 C CNN
F 1 "VCC" H 7267 1323 50  0000 C CNN
F 2 "" H 7250 1150 50  0001 C CNN
F 3 "" H 7250 1150 50  0001 C CNN
	1    7250 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5B9C49FF
P 7250 1350
F 0 "#PWR0111" H 7250 1100 50  0001 C CNN
F 1 "GND" H 7255 1177 50  0000 C CNN
F 2 "" H 7250 1350 50  0001 C CNN
F 3 "" H 7250 1350 50  0001 C CNN
	1    7250 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 1250 7250 1250
Wire Wire Line
	6350 1250 5900 1250
Wire Wire Line
	6350 1150 5900 1150
Wire Wire Line
	6850 1150 7250 1150
Wire Wire Line
	6850 1350 7250 1350
Text Label 7250 1250 0    50   ~ 0
MOSI
Text Label 5900 1250 0    50   ~ 0
SCK
Text Label 5900 1150 0    50   ~ 0
MISO
Text Label 5900 1350 0    50   ~ 0
RST
$Comp
L Device:R R1
U 1 1 5B9C9525
P 5900 3550
F 0 "R1" H 5830 3504 50  0000 R CNN
F 1 "10k" H 5830 3595 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5830 3550 50  0001 C CNN
F 3 "~" H 5900 3550 50  0001 C CNN
	1    5900 3550
	-1   0    0    1   
$EndComp
$Comp
L power:VCC #PWR0112
U 1 1 5B9C9619
P 5900 3400
F 0 "#PWR0112" H 5900 3250 50  0001 C CNN
F 1 "VCC" H 5917 3573 50  0000 C CNN
F 2 "" H 5900 3400 50  0001 C CNN
F 3 "" H 5900 3400 50  0001 C CNN
	1    5900 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3800 5900 3800
Wire Wire Line
	5350 2600 6150 2600
Wire Wire Line
	5350 2700 6150 2700
Wire Wire Line
	5350 2800 6150 2800
Text Label 6150 2800 0    50   ~ 0
SCK
Text Label 6150 2700 0    50   ~ 0
MISO
Text Label 6150 2600 0    50   ~ 0
MOSI
Connection ~ 5900 3800
Wire Wire Line
	5900 3800 6150 3800
Wire Wire Line
	5900 1350 6350 1350
Text Label 6150 3800 0    50   ~ 0
RST
$Comp
L Mechanical:MountingHole_Pad MH1
U 1 1 5B9DA2E3
P 2500 1300
F 0 "MH1" V 2737 1305 50  0000 C CNN
F 1 "VCC" V 2646 1305 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 2500 1300 50  0001 C CNN
F 3 "~" H 2500 1300 50  0001 C CNN
	1    2500 1300
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad MH2
U 1 1 5B9DA4DD
P 2500 1700
F 0 "MH2" V 2737 1705 50  0000 C CNN
F 1 "GND" V 2646 1705 50  0000 C CNN
F 2 "Connector_Pin:Pin_D1.0mm_L10.0mm" H 2500 1700 50  0001 C CNN
F 3 "~" H 2500 1700 50  0001 C CNN
	1    2500 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR0113
U 1 1 5B9DA565
P 2950 1300
F 0 "#PWR0113" H 2950 1150 50  0001 C CNN
F 1 "VCC" H 2967 1473 50  0000 C CNN
F 2 "" H 2950 1300 50  0001 C CNN
F 3 "" H 2950 1300 50  0001 C CNN
	1    2950 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5B9DA59C
P 2950 1700
F 0 "#PWR0114" H 2950 1450 50  0001 C CNN
F 1 "GND" H 2955 1527 50  0000 C CNN
F 2 "" H 2950 1700 50  0001 C CNN
F 3 "" H 2950 1700 50  0001 C CNN
	1    2950 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1300 2950 1300
Wire Wire Line
	2600 1700 2950 1700
Wire Wire Line
	5350 4000 6150 4000
Wire Wire Line
	5350 4100 6150 4100
Wire Wire Line
	5350 4200 6150 4200
Text Label 6150 4200 0    50   ~ 0
LED_B_MCU_OUT
Text Label 6150 4100 0    50   ~ 0
LED_G_MCU_OUT
Text Label 6150 4000 0    50   ~ 0
LED_R_MCU_OUT
Wire Wire Line
	5900 3700 5900 3800
NoConn ~ 4150 2300
NoConn ~ 4150 2500
NoConn ~ 4150 2600
NoConn ~ 5350 4700
NoConn ~ 5350 4600
NoConn ~ 5350 4500
NoConn ~ 5350 4400
NoConn ~ 5350 3700
NoConn ~ 5350 3600
NoConn ~ 5350 3500
NoConn ~ 5350 3400
NoConn ~ 5350 3300
NoConn ~ 5350 3200
NoConn ~ 5350 3000
NoConn ~ 5350 2900
NoConn ~ 5350 2500
NoConn ~ 5350 2400
NoConn ~ 5350 2300
NoConn ~ 4850 2000
Wire Wire Line
	5350 4300 6150 4300
Text Label 6150 4300 0    50   ~ 0
IR_MCU_IN
$Comp
L Device:Q_NMOS_GSD Q1
U 1 1 5BF57757
P 7550 4750
F 0 "Q1" H 7755 4796 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7755 4705 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 7750 4850 50  0001 C CNN
F 3 "~" H 7550 4750 50  0001 C CNN
	1    7550 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q2
U 1 1 5BF57833
P 8850 4750
F 0 "Q2" H 9055 4796 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 9055 4705 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 9050 4850 50  0001 C CNN
F 3 "~" H 8850 4750 50  0001 C CNN
	1    8850 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q3
U 1 1 5BF5796C
P 10200 4750
F 0 "Q3" H 10405 4796 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 10405 4705 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 10400 4850 50  0001 C CNN
F 3 "~" H 10200 4750 50  0001 C CNN
	1    10200 4750
	1    0    0    -1  
$EndComp
$EndSCHEMATC
