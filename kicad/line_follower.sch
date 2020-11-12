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
Wire Wire Line
	1300 6800 1500 6800
Wire Wire Line
	1500 6800 1500 6950
Wire Wire Line
	1800 7200 1800 7300
Wire Wire Line
	3150 7300 2650 7300
Wire Wire Line
	2650 7000 2650 7300
Wire Wire Line
	3150 7300 3150 6600
Wire Wire Line
	3150 7300 3650 7300
Wire Wire Line
	3650 7300 3650 7000
Connection ~ 1800 7300
Connection ~ 2650 7300
Connection ~ 3150 7300
Wire Wire Line
	1350 2050 1450 2050
Wire Wire Line
	1350 2150 1450 2150
Wire Wire Line
	3450 6300 3650 6300
Wire Wire Line
	3650 6700 3650 6300
Wire Wire Line
	3650 6300 3900 6300
Connection ~ 3650 6300
Wire Wire Line
	650  1450 1450 1450
Wire Wire Line
	1450 1550 650  1550
Text Label 650  1550 0    70   ~ 0
LINE_LED2
Wire Wire Line
	1450 1650 650  1650
Text Label 650  1650 0    70   ~ 0
LINE_LED0
Wire Wire Line
	1450 1850 650  1850
Wire Wire Line
	2550 2150 2650 2150
Wire Wire Line
	2550 1150 3350 1150
Text Label 3350 1150 2    70   ~ 0
SDA
Wire Wire Line
	900  2650 1600 2650
Text Label 900  2650 0    70   ~ 0
SDA
Wire Wire Line
	2550 850  3350 850 
Text Label 3350 850  2    70   ~ 0
SCL
Wire Wire Line
	1600 2550 900  2550
Text Label 900  2550 0    70   ~ 0
SCL
Wire Wire Line
	1450 1950 650  1950
Text Label 650  1950 0    70   ~ 0
MOTOR1A
Wire Wire Line
	2550 750  3350 750 
Text Label 3350 750  2    70   ~ 0
MOTOR2A
Wire Wire Line
	2550 1250 3350 1250
Text Label 3350 1250 2    70   ~ 0
MOTOR2B
Text Label 1150 4200 0    70   ~ 0
BUTTON
Wire Wire Line
	2550 1450 3350 1450
Text Label 3350 1450 2    70   ~ 0
BUTTON
Wire Wire Line
	1450 1150 650  1150
Text Label 650  1150 0    70   ~ 0
LINE_SENSOR2
Wire Wire Line
	1300 6700 1500 6700
Wire Wire Line
	1500 6700 1500 6300
Wire Wire Line
	1500 6300 1500 6200
Wire Wire Line
	1800 6400 1800 6300
Wire Wire Line
	2650 6300 2850 6300
Wire Wire Line
	2650 6700 2650 6300
Connection ~ 1500 6300
Connection ~ 1800 6300
Connection ~ 2650 6300
Wire Wire Line
	1800 6900 1800 6800
Wire Wire Line
	1800 6800 1800 6700
Wire Wire Line
	1800 6800 2500 6800
Text Label 2500 6800 2    70   ~ 0
BAT_SENSE
Connection ~ 1800 6800
Wire Wire Line
	2550 1950 3350 1950
Text Label 3350 1950 2    70   ~ 0
BAT_SENSE
Wire Wire Line
	650  1350 1450 1350
Wire Wire Line
	650  1750 1450 1750
Text Label 650  1750 0    70   ~ 0
LINE_SENSOR1
Wire Wire Line
	650  1250 1450 1250
Wire Wire Line
	3350 1750 2550 1750
Text Label 3350 1750 2    70   ~ 0
MOTOR1B
Wire Wire Line
	650  1050 1450 1050
Text Label 650  1050 0    70   ~ 0
LINE_SENSOR4
Wire Wire Line
	1450 850  650  850 
Text Label 650  850  0    70   ~ 0
ENCODER2A
Wire Wire Line
	1450 950  650  950 
Text Label 650  950  0    70   ~ 0
ENCODER2B
Wire Wire Line
	2550 1550 3350 1550
Text Label 3350 1550 2    70   ~ 0
ENCODER1B
Wire Wire Line
	2550 1650 3350 1650
Text Label 3350 1650 2    70   ~ 0
ENCODER1A
Wire Wire Line
	1600 3050 900  3050
Text Label 900  3050 0    70   ~ 0
ACCEL_INT
Wire Wire Line
	2550 1350 3350 1350
Text Label 3350 1350 2    70   ~ 0
ACCEL_INT
Text Notes 750  5750 0    59   ~ 0
Power supply:\n\nTotal current on the 5V rail should be < 250mA at all times.\n\nBoth ESP32-DEVKITV1 and GY-521 have on board 3.3V regulators.\nWe're using the one on ESP32 to power small stuff around the board.\n\nLow dropout voltage regulator was chosen because it\nhas low enough dropout voltage to power the board from 2S LiPo.\nIf using battery with larger voltage, basically any linear regulator is good.\nWatch for pinout differences, though.\n\nWhen connecting board with USB without battery, the motors\nare powered from the USB as well (-2*diode drop)
Text Notes 2600 1050 0    59   ~ 0
UART TX/RX\nconnected to USB
Text Notes 2600 1850 0    59   ~ 0
on-board LED
$Comp
L cube-library:ESP32_DEVKITV1 X1
U 1 1 5EDB02BD
P 2000 1450
F 0 "X1" H 2000 2375 50  0000 C CNN
F 1 "ESP32_DEVKITV1" H 2000 2284 50  0000 C CNN
F 2 "cube-library:ESP32_DEVKITV1" H 1800 2550 50  0001 C CNN
F 3 "https://docs.zerynth.com/latest/official/board.zerynth.doit_esp32/docs/index.html" H 1800 2550 50  0001 C CNN
	1    2000 1450
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L7805 U3
U 1 1 5EE384A3
P 3150 6300
F 0 "U3" H 3150 6542 50  0000 C CNN
F 1 "LF50CDT" H 3150 6451 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 3175 6150 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 3150 6250 50  0001 C CNN
	1    3150 6300
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5EFF085D
P 2050 4200
F 0 "SW1" H 2050 4485 50  0000 C CNN
F 1 "SW_Push" H 2050 4394 50  0000 C CNN
F 2 "cube-library:4.5mm_tact_switch" H 2050 4400 50  0001 C CNN
F 3 "~" H 2050 4400 50  0001 C CNN
	1    2050 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 4200 1850 4200
Wire Wire Line
	2250 4200 2750 4200
Wire Wire Line
	2400 2550 2500 2550
$Comp
L cube-library:GY-521-module X2
U 1 1 5F1A9A8F
P 2000 2500
F 0 "X2" H 2000 2675 50  0000 C CNN
F 1 "GY-521-module" H 2000 2584 50  0000 C CNN
F 2 "cube-library:GY-521-module" H 1950 2650 50  0001 C CNN
F 3 "" H 1950 2650 50  0001 C CNN
	1    2000 2500
	1    0    0    -1  
$EndComp
Text Label 1000 3400 0    70   ~ 0
RANGE
Wire Wire Line
	1700 3400 1000 3400
Wire Wire Line
	1700 3600 1600 3600
$Comp
L cube-library:DISTANCE_SENSOR X3
U 1 1 5F1CF668
P 2000 3500
F 0 "X3" H 1950 3800 50  0000 L CNN
F 1 "DISTANCE_SENSOR" H 1650 3700 50  0000 L CNN
F 2 "cube-library:SolderWirePad1x3_p2.54" H 2000 3950 50  0001 C CNN
F 3 "" H 2000 3950 50  0001 C CNN
	1    2000 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 7300 2650 7300
Wire Wire Line
	1800 6300 2650 6300
Wire Wire Line
	1500 6300 1800 6300
Wire Wire Line
	1500 7300 1800 7300
$Comp
L power:+5V #PWR0101
U 1 1 5F6AF030
P 2500 2550
F 0 "#PWR0101" H 2500 2400 50  0001 C CNN
F 1 "+5V" V 2515 2678 50  0000 L CNN
F 2 "" H 2500 2550 50  0001 C CNN
F 3 "" H 2500 2550 50  0001 C CNN
	1    2500 2550
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0105
U 1 1 5F6B2C5C
P 3900 6300
F 0 "#PWR0105" H 3900 6150 50  0001 C CNN
F 1 "+5V" V 3915 6428 50  0000 L CNN
F 2 "" H 3900 6300 50  0001 C CNN
F 3 "" H 3900 6300 50  0001 C CNN
	1    3900 6300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5F71E04C
P 2750 4200
F 0 "#PWR0114" H 2750 3950 50  0001 C CNN
F 1 "GND" V 2755 4072 50  0000 R CNN
F 2 "" H 2750 4200 50  0001 C CNN
F 3 "" H 2750 4200 50  0001 C CNN
	1    2750 4200
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5F722A39
P 2650 7450
F 0 "#PWR0118" H 2650 7200 50  0001 C CNN
F 1 "GND" H 2655 7277 50  0000 C CNN
F 2 "" H 2650 7450 50  0001 C CNN
F 3 "" H 2650 7450 50  0001 C CNN
	1    2650 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 7300 2650 7450
Text Notes 1550 2950 2    50   ~ 0
AD0 has internal pull down
$Comp
L Device:R R6
U 1 1 5F7CFF22
P 1800 6550
F 0 "R6" H 1870 6596 50  0000 L CNN
F 1 "82k" H 1870 6505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1730 6550 50  0001 C CNN
F 3 "~" H 1800 6550 50  0001 C CNN
	1    1800 6550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5F7D03F7
P 1800 7050
F 0 "R7" H 1870 7096 50  0000 L CNN
F 1 "15k" H 1870 7005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1730 7050 50  0001 C CNN
F 3 "~" H 1800 7050 50  0001 C CNN
	1    1800 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5F85250D
P 2650 6850
F 0 "C10" H 2765 6896 50  0000 L CNN
F 1 "100nF" H 2765 6805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2688 6700 50  0001 C CNN
F 3 "~" H 2650 6850 50  0001 C CNN
	1    2650 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5F85A44C
P 3650 6850
F 0 "C11" H 3765 6896 50  0000 L CNN
F 1 "22uF" H 3765 6805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3688 6700 50  0001 C CNN
F 3 "~" H 3650 6850 50  0001 C CNN
	1    3650 6850
	1    0    0    -1  
$EndComp
NoConn ~ 1600 2750
NoConn ~ 1600 2850
NoConn ~ 1600 2950
NoConn ~ 2550 950 
NoConn ~ 2550 1050
NoConn ~ 1450 750 
NoConn ~ 2550 1850
Wire Wire Line
	2500 2650 2400 2650
$Comp
L power:GND #PWR0113
U 1 1 5FB0D703
P 2500 2650
F 0 "#PWR0113" H 2500 2400 50  0001 C CNN
F 1 "GND" V 2505 2522 50  0000 R CNN
F 2 "" H 2500 2650 50  0001 C CNN
F 3 "" H 2500 2650 50  0001 C CNN
	1    2500 2650
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR0115
U 1 1 5FB0ED44
P 1500 6200
F 0 "#PWR0115" H 1500 6050 50  0001 C CNN
F 1 "+BATT" H 1515 6373 50  0000 C CNN
F 2 "" H 1500 6200 50  0001 C CNN
F 3 "" H 1500 6200 50  0001 C CNN
	1    1500 6200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5FB184FC
P 1350 2050
F 0 "#PWR0106" H 1350 1800 50  0001 C CNN
F 1 "GND" V 1355 1922 50  0000 R CNN
F 2 "" H 1350 2050 50  0001 C CNN
F 3 "" H 1350 2050 50  0001 C CNN
	1    1350 2050
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0112
U 1 1 5FB19722
P 1350 2150
F 0 "#PWR0112" H 1350 2000 50  0001 C CNN
F 1 "+5V" V 1365 2278 50  0000 L CNN
F 2 "" H 1350 2150 50  0001 C CNN
F 3 "" H 1350 2150 50  0001 C CNN
	1    1350 2150
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0126
U 1 1 5FB1A25C
P 2650 2150
F 0 "#PWR0126" H 2650 2000 50  0001 C CNN
F 1 "+3.3V" V 2665 2278 50  0000 L CNN
F 2 "" H 2650 2150 50  0001 C CNN
F 3 "" H 2650 2150 50  0001 C CNN
	1    2650 2150
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0122
U 1 1 5FB2B245
P 1600 3600
F 0 "#PWR0122" H 1600 3450 50  0001 C CNN
F 1 "+5V" V 1615 3728 50  0000 L CNN
F 2 "" H 1600 3600 50  0001 C CNN
F 3 "" H 1600 3600 50  0001 C CNN
	1    1600 3600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5FB4A7A6
P 1600 3700
F 0 "#PWR0102" H 1600 3450 50  0001 C CNN
F 1 "GND" V 1605 3572 50  0000 R CNN
F 2 "" H 1600 3700 50  0001 C CNN
F 3 "" H 1600 3700 50  0001 C CNN
	1    1600 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	1600 3700 1700 3700
$Comp
L power:GND #PWR0107
U 1 1 5FB584EC
P 2650 2050
F 0 "#PWR0107" H 2650 1800 50  0001 C CNN
F 1 "GND" V 2655 1922 50  0000 R CNN
F 2 "" H 2650 2050 50  0001 C CNN
F 3 "" H 2650 2050 50  0001 C CNN
	1    2650 2050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2550 2050 2650 2050
Text Notes 500  750  0    59   ~ 0
has internal pull up
Text Notes 1550 2850 2    50   ~ 0
Secondary I2C is not used
$Comp
L power:+BATT #PWR0123
U 1 1 5FB45C3F
P 5100 3900
F 0 "#PWR0123" H 5100 3750 50  0001 C CNN
F 1 "+BATT" H 5115 4073 50  0000 C CNN
F 2 "" H 5100 3900 50  0001 C CNN
F 3 "" H 5100 3900 50  0001 C CNN
	1    5100 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5FB1C3F7
P 5100 5850
F 0 "#PWR0119" H 5100 5600 50  0001 C CNN
F 1 "GND" H 5105 5677 50  0000 C CNN
F 2 "" H 5100 5850 50  0001 C CNN
F 3 "" H 5100 5850 50  0001 C CNN
	1    5100 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5600 6600 5600
Wire Wire Line
	6600 5500 6700 5500
Connection ~ 5100 5650
Wire Wire Line
	5100 5650 5100 5850
$Comp
L Device:CP C9
U 1 1 5F872EAE
P 5300 4300
F 0 "C9" H 5418 4346 50  0000 L CNN
F 1 "100uF, 25V" H 5418 4255 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x5.8" H 5338 4150 50  0001 C CNN
F 3 "~" H 5300 4300 50  0001 C CNN
	1    5300 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5F868C66
P 6100 4300
F 0 "C7" H 6215 4346 50  0000 L CNN
F 1 "100nF" H 6215 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6138 4150 50  0001 C CNN
F 3 "~" H 6100 4300 50  0001 C CNN
	1    6100 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 5F769F41
P 6100 4650
F 0 "#PWR0124" H 6100 4400 50  0001 C CNN
F 1 "GND" H 6105 4477 50  0000 C CNN
F 2 "" H 6100 4650 50  0001 C CNN
F 3 "" H 6100 4650 50  0001 C CNN
	1    6100 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5F726B92
P 6600 5600
F 0 "#PWR0120" H 6600 5350 50  0001 C CNN
F 1 "GND" V 6605 5472 50  0000 R CNN
F 2 "" H 6600 5600 50  0001 C CNN
F 3 "" H 6600 5600 50  0001 C CNN
	1    6600 5600
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0111
U 1 1 5F6ED7A9
P 6600 5500
F 0 "#PWR0111" H 6600 5350 50  0001 C CNN
F 1 "+3.3V" V 6615 5628 50  0000 L CNN
F 2 "" H 6600 5500 50  0001 C CNN
F 3 "" H 6600 5500 50  0001 C CNN
	1    6600 5500
	0    -1   -1   0   
$EndComp
$Comp
L cube-library:MOTOR_ENC M2
U 1 1 5F092189
P 7300 5400
F 0 "M2" H 7578 5446 50  0000 L CNN
F 1 "MOTOR_ENC" H 7578 5355 50  0000 L CNN
F 2 "cube-library:SolderWirePad2x3_p2.54_staggered" H 7300 5700 50  0001 C CNN
F 3 "" H 7300 5600 50  0001 C CNN
	1    7300 5400
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:A4950E U2
U 1 1 5EDD061A
P 5100 5150
F 0 "U2" H 5100 5731 50  0000 C CNN
F 1 "A4950E" H 5100 5640 50  0000 C CNN
F 2 "cube-library:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm_HandSolder" H 5100 4600 50  0001 C CNN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A4950-Datasheet.ashx" H 4800 5500 50  0001 C CNN
	1    5100 5150
	1    0    0    -1  
$EndComp
Text Label 5900 5300 0    70   ~ 0
ENCODER2B
Wire Wire Line
	6700 5300 5900 5300
Text Label 5900 5200 0    70   ~ 0
ENCODER2A
Wire Wire Line
	6700 5200 5900 5200
Wire Wire Line
	7300 5850 7300 5800
Wire Wire Line
	5800 5850 5800 5050
Wire Wire Line
	7300 5850 5800 5850
Wire Wire Line
	5500 5050 5800 5050
Wire Wire Line
	7300 4950 7300 5000
Wire Wire Line
	5500 4950 7300 4950
Connection ~ 5100 4050
Connection ~ 5300 4050
Wire Wire Line
	5300 4050 5100 4050
Wire Wire Line
	6100 4050 6100 4150
Wire Wire Line
	5300 4050 6100 4050
Wire Wire Line
	5300 4150 5300 4050
Wire Wire Line
	5100 4050 5100 4750
Wire Wire Line
	5100 3900 5100 4050
Text Label 4200 5050 0    70   ~ 0
MOTOR2B
Wire Wire Line
	4700 5050 4200 5050
Text Label 4200 4950 0    70   ~ 0
MOTOR2A
Wire Wire Line
	4700 4950 4200 4950
Wire Wire Line
	4600 5250 4700 5250
Connection ~ 5200 5650
Wire Wire Line
	5200 5550 5200 5650
Wire Wire Line
	5100 5550 5100 5650
Wire Wire Line
	5200 5650 5100 5650
Wire Wire Line
	5600 5650 5200 5650
Wire Wire Line
	5600 5150 5600 5650
Wire Wire Line
	5500 5150 5600 5150
Connection ~ 6100 4550
Wire Wire Line
	5300 4550 6100 4550
Wire Wire Line
	5300 4450 5300 4550
Wire Wire Line
	6100 4550 6100 4650
Wire Wire Line
	6100 4450 6100 4550
Text Notes 4050 900  0    59   ~ 0
H bridges:\n\nWe are not using current limiting,\n3.3V is just tied to VREF because it was close for routing
$Comp
L power:GND #PWR0109
U 1 1 5FB0E3F3
P 5100 3100
F 0 "#PWR0109" H 5100 2850 50  0001 C CNN
F 1 "GND" H 5105 2927 50  0000 C CNN
F 2 "" H 5100 3100 50  0001 C CNN
F 3 "" H 5100 3100 50  0001 C CNN
	1    5100 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5FB0C291
P 6600 2900
F 0 "#PWR0116" H 6600 2650 50  0001 C CNN
F 1 "GND" V 6605 2772 50  0000 R CNN
F 2 "" H 6600 2900 50  0001 C CNN
F 3 "" H 6600 2900 50  0001 C CNN
	1    6600 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	6600 2900 6700 2900
Wire Wire Line
	6600 2800 6700 2800
$Comp
L Device:CP C8
U 1 1 5F86EBBA
P 5300 1600
F 0 "C8" H 5418 1646 50  0000 L CNN
F 1 "100uF, 25V" H 5418 1555 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x5.8" H 5338 1450 50  0001 C CNN
F 3 "~" H 5300 1600 50  0001 C CNN
	1    5300 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5F85D144
P 6100 1600
F 0 "C6" H 6215 1646 50  0000 L CNN
F 1 "100nF" H 6215 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6138 1450 50  0001 C CNN
F 3 "~" H 6100 1600 50  0001 C CNN
	1    6100 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2550 4700 2550
$Comp
L power:+BATT #PWR0125
U 1 1 5F783751
P 5100 1200
F 0 "#PWR0125" H 5100 1050 50  0001 C CNN
F 1 "+BATT" H 5115 1373 50  0000 C CNN
F 2 "" H 5100 1200 50  0001 C CNN
F 3 "" H 5100 1200 50  0001 C CNN
	1    5100 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5F7225A2
P 6100 1950
F 0 "#PWR0117" H 6100 1700 50  0001 C CNN
F 1 "GND" H 6105 1777 50  0000 C CNN
F 2 "" H 6100 1950 50  0001 C CNN
F 3 "" H 6100 1950 50  0001 C CNN
	1    6100 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0110
U 1 1 5F6E7E75
P 6600 2800
F 0 "#PWR0110" H 6600 2650 50  0001 C CNN
F 1 "+3.3V" V 6615 2928 50  0000 L CNN
F 2 "" H 6600 2800 50  0001 C CNN
F 3 "" H 6600 2800 50  0001 C CNN
	1    6600 2800
	0    -1   -1   0   
$EndComp
$Comp
L cube-library:MOTOR_ENC M1
U 1 1 5F08FA5D
P 7300 2700
F 0 "M1" H 7578 2746 50  0000 L CNN
F 1 "MOTOR_ENC" H 7578 2655 50  0000 L CNN
F 2 "cube-library:SolderWirePad2x3_p2.54_staggered" H 7300 3000 50  0001 C CNN
F 3 "" H 7300 2900 50  0001 C CNN
	1    7300 2700
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:A4950E U1
U 1 1 5EDCF835
P 5100 2450
F 0 "U1" H 5100 3031 50  0000 C CNN
F 1 "A4950E" H 5100 2940 50  0000 C CNN
F 2 "cube-library:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm_HandSolder" H 5100 1900 50  0001 C CNN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A4950-Datasheet.ashx" H 4800 2800 50  0001 C CNN
	1    5100 2450
	1    0    0    -1  
$EndComp
Text Label 5900 2600 0    70   ~ 0
ENCODER1A
Wire Wire Line
	6700 2600 5900 2600
Text Label 5900 2500 0    70   ~ 0
ENCODER1B
Wire Wire Line
	6700 2500 5900 2500
Wire Wire Line
	7300 2250 7300 2300
Wire Wire Line
	5500 2250 7300 2250
Wire Wire Line
	7300 3150 7300 3100
Wire Wire Line
	5800 3150 7300 3150
Wire Wire Line
	5800 2350 5800 3150
Wire Wire Line
	5500 2350 5800 2350
Wire Wire Line
	5300 1450 5300 1350
Wire Wire Line
	5100 2050 5100 1350
Wire Wire Line
	5100 1200 5100 1350
Connection ~ 5100 1350
Wire Wire Line
	5300 1350 5100 1350
Connection ~ 5300 1350
Wire Wire Line
	6100 1350 5300 1350
Wire Wire Line
	6100 1450 6100 1350
Text Label 4200 2350 0    70   ~ 0
MOTOR1A
Wire Wire Line
	4700 2350 4200 2350
Text Label 4200 2250 0    70   ~ 0
MOTOR1B
Wire Wire Line
	4700 2250 4200 2250
Wire Wire Line
	5100 2950 5100 3100
Connection ~ 5100 2950
Wire Wire Line
	5200 2950 5100 2950
Wire Wire Line
	5200 2850 5200 2950
Connection ~ 5200 2950
Wire Wire Line
	5600 2950 5200 2950
Wire Wire Line
	5500 2450 5600 2450
Wire Wire Line
	5600 2450 5600 2950
Wire Wire Line
	5300 1850 6100 1850
Wire Wire Line
	5300 1750 5300 1850
Wire Wire Line
	6100 1850 6100 1750
Connection ~ 6100 1850
Wire Wire Line
	6100 1950 6100 1850
Text Notes 8000 5150 0    59   ~ 0
Line sensor LEDs\n\nCharlieplexing leds, using ESP32 GPIO drive strength to limit current.\nShould be safe even at max drive strength at about 30mA per LED.\n\nSee https://www.esp32.com/viewtopic.php?f=12&t=5840#p25550
Text Notes 8050 600  0    59   ~ 0
Line sensor
Wire Wire Line
	9900 6200 9900 6300
Connection ~ 9900 6300
Wire Wire Line
	9500 6300 9900 6300
Wire Wire Line
	9500 6200 9500 6300
Connection ~ 9500 6300
Wire Wire Line
	8850 6300 9500 6300
Wire Wire Line
	10700 6000 10700 6300
Wire Wire Line
	9900 6300 10300 6300
Wire Wire Line
	10300 6300 10700 6300
Connection ~ 10300 6300
Wire Wire Line
	10300 6000 10300 6300
Wire Wire Line
	10700 5300 10700 5700
Wire Wire Line
	10300 5300 10700 5300
Wire Wire Line
	10300 5700 10300 5300
Connection ~ 10300 5300
Wire Wire Line
	9900 5300 10300 5300
Wire Wire Line
	9900 5400 9900 5300
Wire Wire Line
	9900 5900 9900 5800
Wire Wire Line
	9900 5700 9900 5800
Connection ~ 9900 5800
Wire Wire Line
	9500 5800 9900 5800
Connection ~ 9900 5300
Wire Wire Line
	9500 5300 9900 5300
Wire Wire Line
	9500 5400 9500 5300
Connection ~ 9500 5300
Wire Wire Line
	8850 5300 9500 5300
$Comp
L Device:LED D4
U 1 1 5F6A8D59
P 10700 5850
F 0 "D4" V 10739 5732 50  0000 R CNN
F 1 "APT2012EC" V 10648 5732 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10700 5850 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 10700 5850 50  0001 C CNN
	1    10700 5850
	0    1    1    0   
$EndComp
$Comp
L Device:LED D3
U 1 1 5F6A8692
P 10300 5850
F 0 "D3" V 10247 5929 50  0000 L CNN
F 1 "APT2012EC" V 10338 5929 50  0000 L CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10300 5850 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 10300 5850 50  0001 C CNN
	1    10300 5850
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D6
U 1 1 5F6A2E93
P 9900 6050
F 0 "D6" V 9847 6129 50  0000 L CNN
F 1 "APT2012EC" V 9938 6129 50  0000 L CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9900 6050 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 9900 6050 50  0001 C CNN
	1    9900 6050
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 5F6A2946
P 9500 6050
F 0 "D5" V 9539 5932 50  0000 R CNN
F 1 "APT2012EC" V 9448 5932 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9500 6050 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 9500 6050 50  0001 C CNN
	1    9500 6050
	0    1    1    0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5F69D55B
P 9900 5550
F 0 "D2" V 9939 5432 50  0000 R CNN
F 1 "APT2012EC" V 9848 5432 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9900 5550 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 9900 5550 50  0001 C CNN
	1    9900 5550
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5F69B5F1
P 9500 5550
F 0 "D1" V 9447 5629 50  0000 L CNN
F 1 "APT2012EC" V 9538 5629 50  0000 L CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9500 5550 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 9500 5550 50  0001 C CNN
	1    9500 5550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9500 5800 9500 5900
Wire Wire Line
	9500 5700 9500 5800
Connection ~ 9500 5800
Wire Wire Line
	8850 5800 9500 5800
$Comp
L Device:Q_Photo_NPN Q5
U 1 1 5F8D10CE
P 9150 4100
F 0 "Q5" V 8835 4100 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8926 4100 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9350 4200 50  0001 C CNN
F 3 "~" H 9150 4100 50  0001 C CNN
	1    9150 4100
	0    -1   1    0   
$EndComp
$Comp
L Device:Q_Photo_NPN Q4
U 1 1 5F8D075D
P 9150 3300
F 0 "Q4" V 8835 3300 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8926 3300 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9350 3400 50  0001 C CNN
F 3 "~" H 9150 3300 50  0001 C CNN
	1    9150 3300
	0    -1   1    0   
$EndComp
$Comp
L Device:Q_Photo_NPN Q3
U 1 1 5F8D019A
P 9150 2500
F 0 "Q3" V 8835 2500 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8926 2500 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9350 2600 50  0001 C CNN
F 3 "~" H 9150 2500 50  0001 C CNN
	1    9150 2500
	0    -1   1    0   
$EndComp
$Comp
L Device:Q_Photo_NPN Q2
U 1 1 5F8CEDFE
P 9150 1700
F 0 "Q2" V 8835 1700 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8926 1700 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9350 1800 50  0001 C CNN
F 3 "~" H 9150 1700 50  0001 C CNN
	1    9150 1700
	0    -1   1    0   
$EndComp
$Comp
L Device:Q_Photo_NPN Q1
U 1 1 5F883D85
P 9150 900
F 0 "Q1" V 8835 900 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8926 900 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9350 1000 50  0001 C CNN
F 3 "~" H 9150 900 50  0001 C CNN
	1    9150 900 
	0    -1   1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5F810985
P 9900 4200
F 0 "R5" V 9693 4200 50  0000 C CNN
F 1 "5k6" V 9784 4200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9830 4200 50  0001 C CNN
F 3 "~" H 9900 4200 50  0001 C CNN
	1    9900 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5F810521
P 9900 3400
F 0 "R4" V 9693 3400 50  0000 C CNN
F 1 "5k6" V 9784 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9830 3400 50  0001 C CNN
F 3 "~" H 9900 3400 50  0001 C CNN
	1    9900 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5F80FD5B
P 9900 1800
F 0 "R2" V 9693 1800 50  0000 C CNN
F 1 "5k6" V 9784 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9830 1800 50  0001 C CNN
F 3 "~" H 9900 1800 50  0001 C CNN
	1    9900 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5F7F32DB
P 9900 1000
F 0 "R1" V 9693 1000 50  0000 C CNN
F 1 "5k6" V 9784 1000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9830 1000 50  0001 C CNN
F 3 "~" H 9900 1000 50  0001 C CNN
	1    9900 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5F72ACF9
P 10250 4500
F 0 "#PWR0121" H 10250 4250 50  0001 C CNN
F 1 "GND" H 10255 4327 50  0000 C CNN
F 2 "" H 10250 4500 50  0001 C CNN
F 3 "" H 10250 4500 50  0001 C CNN
	1    10250 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5F6B9B15
P 8650 1000
F 0 "#PWR0108" H 8650 850 50  0001 C CNN
F 1 "+3.3V" V 8665 1128 50  0000 L CNN
F 2 "" H 8650 1000 50  0001 C CNN
F 3 "" H 8650 1000 50  0001 C CNN
	1    8650 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9350 1000 9550 1000
Wire Wire Line
	9550 800  10600 800 
Wire Wire Line
	9550 1000 9550 800 
Connection ~ 9550 1000
Wire Wire Line
	9550 1200 9550 1000
Wire Wire Line
	9750 1200 9550 1200
Wire Wire Line
	9550 1000 9750 1000
Wire Wire Line
	9350 1800 9550 1800
Wire Wire Line
	9550 1600 10600 1600
Wire Wire Line
	9550 1800 9550 1600
Connection ~ 9550 1800
Wire Wire Line
	9550 2000 9550 1800
Wire Wire Line
	9750 2000 9550 2000
Wire Wire Line
	9550 1800 9750 1800
Wire Wire Line
	9350 2600 9550 2600
Wire Wire Line
	9550 2400 10600 2400
Wire Wire Line
	9550 2600 9550 2400
Connection ~ 9550 2600
Wire Wire Line
	9550 2800 9550 2600
Wire Wire Line
	9750 2800 9550 2800
Wire Wire Line
	9550 2600 9750 2600
Wire Wire Line
	9550 3400 9350 3400
Wire Wire Line
	9550 3200 10600 3200
Wire Wire Line
	9550 3400 9550 3200
Connection ~ 9550 3400
Wire Wire Line
	9550 3600 9550 3400
Wire Wire Line
	9750 3600 9550 3600
Wire Wire Line
	9550 3400 9750 3400
Wire Wire Line
	9550 4200 9350 4200
Wire Wire Line
	9550 4000 10600 4000
Wire Wire Line
	9550 4200 9550 4000
Wire Wire Line
	9550 4400 9750 4400
Connection ~ 9550 4200
Wire Wire Line
	9550 4200 9550 4400
Wire Wire Line
	9750 4200 9550 4200
Wire Wire Line
	8950 2600 8850 2600
Wire Wire Line
	8950 4200 8850 4200
Wire Wire Line
	8950 3400 8850 3400
Wire Wire Line
	8950 1800 8850 1800
Wire Wire Line
	8850 1000 8950 1000
Wire Wire Line
	8650 1000 8850 1000
Connection ~ 8850 1000
Wire Wire Line
	8850 1800 8850 1000
Connection ~ 8850 1800
Wire Wire Line
	8850 2600 8850 1800
Connection ~ 8850 2600
Wire Wire Line
	8850 3400 8850 2600
Connection ~ 8850 3400
Wire Wire Line
	8850 4200 8850 3400
Wire Wire Line
	10250 1000 10050 1000
Wire Wire Line
	10250 1000 10250 1200
Connection ~ 10250 1200
Wire Wire Line
	10050 1200 10250 1200
Wire Wire Line
	10250 1800 10250 1200
Connection ~ 10250 1800
Wire Wire Line
	10250 1800 10050 1800
Wire Wire Line
	10250 2000 10250 1800
Connection ~ 10250 2000
Wire Wire Line
	10050 2000 10250 2000
Wire Wire Line
	10250 2600 10250 2000
Connection ~ 10250 2600
Wire Wire Line
	10250 2600 10050 2600
Wire Wire Line
	10250 2800 10250 2600
Connection ~ 10250 2800
Wire Wire Line
	10050 2800 10250 2800
Wire Wire Line
	10250 3400 10250 2800
Connection ~ 10250 3400
Wire Wire Line
	10250 3400 10050 3400
Wire Wire Line
	10250 3600 10250 3400
Connection ~ 10250 3600
Wire Wire Line
	10050 3600 10250 3600
Wire Wire Line
	10250 4500 10250 4400
Connection ~ 10250 4400
Wire Wire Line
	10250 4400 10050 4400
Wire Wire Line
	10250 4200 10250 3600
Wire Wire Line
	10250 4200 10250 4400
Connection ~ 10250 4200
Wire Wire Line
	10050 4200 10250 4200
$Comp
L Mechanical:Heatsink_Pad HS1
U 1 1 5F1F9B2A
P 6600 1350
F 0 "HS1" H 6741 1389 50  0000 L CNN
F 1 "Heatsink_Pad" H 6741 1298 50  0000 L CNN
F 2 "cube-library:3x3_thermal_vias" H 6612 1300 50  0001 C CNN
F 3 "~" H 6612 1300 50  0001 C CNN
	1    6600 1350
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Heatsink_Pad HS4
U 1 1 5F204F60
P 6550 4600
F 0 "HS4" H 6691 4639 50  0000 L CNN
F 1 "Heatsink_Pad" H 6691 4548 50  0000 L CNN
F 2 "cube-library:3x3_thermal_vias" H 6562 4550 50  0001 C CNN
F 3 "~" H 6562 4550 50  0001 C CNN
	1    6550 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 2950 5100 2850
$Comp
L power:GND #PWR0127
U 1 1 5F2722C1
P 6600 1450
F 0 "#PWR0127" H 6600 1200 50  0001 C CNN
F 1 "GND" H 6605 1277 50  0000 C CNN
F 2 "" H 6600 1450 50  0001 C CNN
F 3 "" H 6600 1450 50  0001 C CNN
	1    6600 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5F27922E
P 6550 4700
F 0 "#PWR0128" H 6550 4450 50  0001 C CNN
F 1 "GND" H 6555 4527 50  0000 C CNN
F 2 "" H 6550 4700 50  0001 C CNN
F 3 "" H 6550 4700 50  0001 C CNN
	1    6550 4700
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Heatsink_Pad HS3
U 1 1 5F292D49
P 6550 4050
F 0 "HS3" H 6691 4089 50  0000 L CNN
F 1 "Heatsink_Pad" H 6691 3998 50  0000 L CNN
F 2 "cube-library:3x3_thermal_vias" H 6562 4000 50  0001 C CNN
F 3 "~" H 6562 4000 50  0001 C CNN
	1    6550 4050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Heatsink_Pad HS2
U 1 1 5F29358A
P 6600 1850
F 0 "HS2" H 6741 1889 50  0000 L CNN
F 1 "Heatsink_Pad" H 6741 1798 50  0000 L CNN
F 2 "cube-library:3x3_thermal_vias" H 6612 1800 50  0001 C CNN
F 3 "~" H 6612 1800 50  0001 C CNN
	1    6600 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 5F29604C
P 6600 1950
F 0 "#PWR0129" H 6600 1700 50  0001 C CNN
F 1 "GND" H 6605 1777 50  0000 C CNN
F 2 "" H 6600 1950 50  0001 C CNN
F 3 "" H 6600 1950 50  0001 C CNN
	1    6600 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 5F29648E
P 6550 4150
F 0 "#PWR0130" H 6550 3900 50  0001 C CNN
F 1 "GND" H 6555 3977 50  0000 C CNN
F 2 "" H 6550 4150 50  0001 C CNN
F 3 "" H 6550 4150 50  0001 C CNN
	1    6550 4150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5F2EDDA3
P 4600 2550
F 0 "#PWR0103" H 4600 2400 50  0001 C CNN
F 1 "+3.3V" V 4615 2678 50  0000 L CNN
F 2 "" H 4600 2550 50  0001 C CNN
F 3 "" H 4600 2550 50  0001 C CNN
	1    4600 2550
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5F2F0575
P 4600 5250
F 0 "#PWR0104" H 4600 5100 50  0001 C CNN
F 1 "+3.3V" V 4615 5378 50  0000 L CNN
F 2 "" H 4600 5250 50  0001 C CNN
F 3 "" H 4600 5250 50  0001 C CNN
	1    4600 5250
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 5F2C31EF
P 4750 6500
F 0 "H1" H 4850 6546 50  0000 L CNN
F 1 "MountingHole" H 4850 6455 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 4750 6500 50  0001 C CNN
F 3 "~" H 4750 6500 50  0001 C CNN
	1    4750 6500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5F2CFC10
P 4750 6700
F 0 "H2" H 4850 6746 50  0000 L CNN
F 1 "MountingHole" H 4850 6655 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 4750 6700 50  0001 C CNN
F 3 "~" H 4750 6700 50  0001 C CNN
	1    4750 6700
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5F2CFF59
P 4750 6900
F 0 "H3" H 4850 6946 50  0000 L CNN
F 1 "MountingHole" H 4850 6855 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 4750 6900 50  0001 C CNN
F 3 "~" H 4750 6900 50  0001 C CNN
	1    4750 6900
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5F2D0257
P 4750 7100
F 0 "H4" H 4850 7146 50  0000 L CNN
F 1 "MountingHole" H 4850 7055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 4750 7100 50  0001 C CNN
F 3 "~" H 4750 7100 50  0001 C CNN
	1    4750 7100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H5
U 1 1 5F2D04F5
P 4750 7300
F 0 "H5" H 4850 7346 50  0000 L CNN
F 1 "MountingHole" H 4850 7255 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 4750 7300 50  0001 C CNN
F 3 "~" H 4750 7300 50  0001 C CNN
	1    4750 7300
	1    0    0    -1  
$EndComp
$Comp
L cube-library:XT30PW-M J1
U 1 1 5EDD27F3
P 1100 6550
F 0 "J1" H 1163 6625 50  0000 C CNN
F 1 "XT30PW-M" H 1163 6534 50  0000 C CNN
F 2 "cube-library:XT30PW-M" H 1200 5900 50  0001 C CNN
F 3 "https://www.tme.eu/Document/ce4077e36b79046da520ca73227e15de/XT30PW%20SPEC.pdf" H 1100 6550 50  0001 C CNN
	1    1100 6550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1300 6950 1500 6950
Connection ~ 1500 6950
Wire Wire Line
	1500 6950 1500 7050
Wire Wire Line
	1300 7050 1500 7050
Connection ~ 1500 7050
Wire Wire Line
	1500 7050 1500 7300
Text Notes 1150 4300 0    50   ~ 0
GPIO5 has a pull-up on X1
Text Label 650  1350 0    70   ~ 0
LINE_SENSOR0
Text Label 650  1250 0    70   ~ 0
LINE_LED1
$Comp
L Device:R R3
U 1 1 5F8100EF
P 9900 2600
F 0 "R3" V 9693 2600 50  0000 C CNN
F 1 "5k6" V 9784 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9830 2600 50  0001 C CNN
F 3 "~" H 9900 2600 50  0001 C CNN
	1    9900 2600
	0    1    1    0   
$EndComp
Text Label 650  1450 0    70   ~ 0
LINE_SENSOR3
$Comp
L Device:C C5
U 1 1 5F8649E1
P 9900 4400
F 0 "C5" V 9648 4400 50  0000 C CNN
F 1 "100nF" V 9739 4400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9938 4250 50  0001 C CNN
F 3 "~" H 9900 4400 50  0001 C CNN
	1    9900 4400
	0    1    1    0   
$EndComp
$Comp
L Device:C C4
U 1 1 5F864455
P 9900 3600
F 0 "C4" V 9648 3600 50  0000 C CNN
F 1 "100nF" V 9739 3600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9938 3450 50  0001 C CNN
F 3 "~" H 9900 3600 50  0001 C CNN
	1    9900 3600
	0    1    1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 5F863E50
P 9900 2800
F 0 "C3" V 9648 2800 50  0000 C CNN
F 1 "100nF" V 9739 2800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9938 2650 50  0001 C CNN
F 3 "~" H 9900 2800 50  0001 C CNN
	1    9900 2800
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5F863769
P 9900 2000
F 0 "C2" V 9648 2000 50  0000 C CNN
F 1 "100nF" V 9739 2000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9938 1850 50  0001 C CNN
F 3 "~" H 9900 2000 50  0001 C CNN
	1    9900 2000
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 5F862D5E
P 9900 1200
F 0 "C1" V 9648 1200 50  0000 C CNN
F 1 "100nF" V 9739 1200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9938 1050 50  0001 C CNN
F 3 "~" H 9900 1200 50  0001 C CNN
	1    9900 1200
	0    1    1    0   
$EndComp
Text Label 650  1850 0    70   ~ 0
RANGE
Text Label 8850 5300 0    50   ~ 0
LINE_LED0
Text Label 8850 5800 0    50   ~ 0
LINE_LED1
Text Label 8850 6300 0    50   ~ 0
LINE_LED2
Text Label 10600 800  0    50   ~ 0
LINE_SENSOR0
Text Label 10600 1600 0    50   ~ 0
LINE_SENSOR1
Text Label 10600 2400 0    50   ~ 0
LINE_SENSOR2
Text Label 10600 3200 0    50   ~ 0
LINE_SENSOR3
Text Label 10600 4000 0    50   ~ 0
LINE_SENSOR4
$EndSCHEMATC
