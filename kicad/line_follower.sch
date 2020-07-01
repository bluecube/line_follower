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
	1000 7100 1200 7100
Wire Wire Line
	1200 7100 1200 7600
Wire Wire Line
	1500 7500 1500 7600
Wire Wire Line
	2850 7600 2350 7600
Wire Wire Line
	2350 7300 2350 7600
Wire Wire Line
	2850 7600 2850 6900
Wire Wire Line
	2850 7600 3350 7600
Wire Wire Line
	3350 7600 3350 7300
Connection ~ 1500 7600
Connection ~ 2350 7600
Connection ~ 2850 7600
Wire Wire Line
	1250 2050 1350 2050
Wire Wire Line
	1250 2150 1350 2150
Wire Wire Line
	3150 6600 3350 6600
Wire Wire Line
	3350 7000 3350 6600
Wire Wire Line
	3350 6600 3600 6600
Connection ~ 3350 6600
Wire Wire Line
	550  1450 1350 1450
Text Label 550  1450 0    70   ~ 0
LINE_LED0
Wire Wire Line
	1350 1550 550  1550
Text Label 550  1550 0    70   ~ 0
LINE_LED1
Wire Wire Line
	1350 1650 550  1650
Text Label 550  1650 0    70   ~ 0
LINE_LED2
Wire Wire Line
	1350 1850 550  1850
Text Label 550  1850 0    70   ~ 0
MOTOR2A
Wire Wire Line
	2450 2150 2550 2150
Wire Wire Line
	2450 1150 3250 1150
Text Label 3250 1150 2    70   ~ 0
SDA
Wire Wire Line
	800  2800 1500 2800
Text Label 800  2800 0    70   ~ 0
SDA
Wire Wire Line
	2450 850  3250 850 
Text Label 3250 850  2    70   ~ 0
SCL
Wire Wire Line
	1500 2700 800  2700
Text Label 800  2700 0    70   ~ 0
SCL
Wire Wire Line
	1350 1950 550  1950
Text Label 550  1950 0    70   ~ 0
MOTOR2B
Wire Wire Line
	2450 750  3250 750 
Text Label 3250 750  2    70   ~ 0
MOTOR1B
Wire Wire Line
	2450 1250 3250 1250
Text Label 3250 1250 2    70   ~ 0
MOTOR1A
Text Label 1000 4700 0    70   ~ 0
BUTTON
Wire Wire Line
	2450 1450 3250 1450
Text Label 3250 1450 2    70   ~ 0
BUTTON
Wire Wire Line
	1350 1150 550  1150
Text Label 550  1150 0    70   ~ 0
RANGE
Wire Wire Line
	1000 7000 1200 7000
Wire Wire Line
	1200 7000 1200 6600
Wire Wire Line
	1200 6600 1200 6500
Wire Wire Line
	1500 6700 1500 6600
Wire Wire Line
	2350 6600 2550 6600
Wire Wire Line
	2350 7000 2350 6600
Connection ~ 1200 6600
Connection ~ 1500 6600
Connection ~ 2350 6600
Wire Wire Line
	1500 7200 1500 7100
Wire Wire Line
	1500 7100 1500 7000
Wire Wire Line
	1500 7100 2200 7100
Text Label 2200 7100 2    70   ~ 0
BAT_SENSE
Connection ~ 1500 7100
Wire Wire Line
	2450 1950 3250 1950
Text Label 3250 1950 2    70   ~ 0
BAT_SENSE
Wire Wire Line
	550  1350 1350 1350
Text Label 550  1350 0    70   ~ 0
LINE_SENSOR4
Wire Wire Line
	550  1750 1350 1750
Text Label 550  1750 0    70   ~ 0
LINE_SENSOR3
Wire Wire Line
	550  1250 1350 1250
Text Label 550  1250 0    70   ~ 0
LINE_SENSOR2
Wire Wire Line
	3250 1750 2450 1750
Text Label 3250 1750 2    70   ~ 0
LINE_SENSOR1
Wire Wire Line
	550  1050 1350 1050
Text Label 550  1050 0    70   ~ 0
LINE_SENSOR0
Wire Wire Line
	1350 850  550  850 
Text Label 550  850  0    70   ~ 0
ENCODER1B
Wire Wire Line
	1350 950  550  950 
Text Label 550  950  0    70   ~ 0
ENCODER1A
Wire Wire Line
	2450 1550 3250 1550
Text Label 3250 1550 2    70   ~ 0
ENCODER2A
Wire Wire Line
	2450 1650 3250 1650
Text Label 3250 1650 2    70   ~ 0
ENCODER2B
Wire Wire Line
	1500 3200 800  3200
Text Label 800  3200 0    70   ~ 0
ACCEL_INT
Wire Wire Line
	2450 1350 3250 1350
Text Label 3250 1350 2    70   ~ 0
ACCEL_INT
Text Notes 450  6050 0    59   ~ 0
Power supply:\n\nBoth ESP32-DEVKITV1 and GY-521 have on board 3.3V regulators.\nWe're using the one on ESP32 to power small stuff around the board.\n\nNCV4274CDT low dropout voltage regulator was chosen because it\nhas low enough dropout voltage to power the board from 2S LiPo,\nand has an internal diode in series, so that back powering the board\nfrom USB is safe (Powers everything except the motors).
Text Notes 2550 1050 0    59   ~ 0
GPIO1 & 3 are UART TX/RX connected to USB
Text Notes 2550 1850 0    59   ~ 0
GPIO2 has the on-board LED
$Comp
L cube-library:ESP32_DEVKITV1 X1
U 1 1 5EDB02BD
P 1900 1450
F 0 "X1" H 1900 2375 50  0000 C CNN
F 1 "ESP32_DEVKITV1" H 1900 2284 50  0000 C CNN
F 2 "cube-library:ESP32_DEVKITV1" H 1700 2550 50  0001 C CNN
F 3 "https://docs.zerynth.com/latest/official/board.zerynth.doit_esp32/docs/index.html" H 1700 2550 50  0001 C CNN
	1    1900 1450
	1    0    0    -1  
$EndComp
$Comp
L cube-library:XT30PW-M J1
U 1 1 5EDD27F3
P 800 6850
F 0 "J1" H 863 6925 50  0000 C CNN
F 1 "XT30PW-M" H 863 6834 50  0000 C CNN
F 2 "cube-library:XT30PW-M" H 900 6200 50  0001 C CNN
F 3 "https://www.tme.eu/Document/ce4077e36b79046da520ca73227e15de/XT30PW%20SPEC.pdf" H 800 6850 50  0001 C CNN
	1    800  6850
	-1   0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L7805 U3
U 1 1 5EE384A3
P 2850 6600
F 0 "U3" H 2850 6842 50  0000 C CNN
F 1 "NCV4274CDT50RKG" H 2850 6751 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 2875 6450 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 2850 6550 50  0001 C CNN
	1    2850 6600
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5EFF085D
P 1900 4700
F 0 "SW1" H 1900 4985 50  0000 C CNN
F 1 "SW_Push" H 1900 4894 50  0000 C CNN
F 2 "cube-library:4.5mm_tact_switch" H 1900 4900 50  0001 C CNN
F 3 "~" H 1900 4900 50  0001 C CNN
	1    1900 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 4700 1700 4700
Wire Wire Line
	2100 4700 2600 4700
Wire Wire Line
	2300 2700 2400 2700
$Comp
L cube-library:GY-521-module X2
U 1 1 5F1A9A8F
P 1900 2650
F 0 "X2" H 1900 2825 50  0000 C CNN
F 1 "GY-521-module" H 1900 2734 50  0000 C CNN
F 2 "cube-library:GY-521-module" H 1850 2800 50  0001 C CNN
F 3 "" H 1850 2800 50  0001 C CNN
	1    1900 2650
	1    0    0    -1  
$EndComp
Text Label 850  3700 0    70   ~ 0
RANGE
Wire Wire Line
	1550 3700 850  3700
Wire Wire Line
	1550 3900 1450 3900
$Comp
L cube-library:DISTANCE_SENSOR X3
U 1 1 5F1CF668
P 1850 3800
F 0 "X3" H 1800 4100 50  0000 L CNN
F 1 "DISTANCE_SENSOR" H 1500 4000 50  0000 L CNN
F 2 "cube-library:SolderWirePad1x3_p2.54" H 1850 4250 50  0001 C CNN
F 3 "" H 1850 4250 50  0001 C CNN
	1    1850 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 7600 2350 7600
Wire Wire Line
	1500 6600 2350 6600
Wire Wire Line
	1200 6600 1500 6600
Wire Wire Line
	1200 7600 1500 7600
$Comp
L power:+5V #PWR0101
U 1 1 5F6AF030
P 2400 2700
F 0 "#PWR0101" H 2400 2550 50  0001 C CNN
F 1 "+5V" V 2415 2828 50  0000 L CNN
F 2 "" H 2400 2700 50  0001 C CNN
F 3 "" H 2400 2700 50  0001 C CNN
	1    2400 2700
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0105
U 1 1 5F6B2C5C
P 3600 6600
F 0 "#PWR0105" H 3600 6450 50  0001 C CNN
F 1 "+5V" V 3615 6728 50  0000 L CNN
F 2 "" H 3600 6600 50  0001 C CNN
F 3 "" H 3600 6600 50  0001 C CNN
	1    3600 6600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5F71E04C
P 2600 4700
F 0 "#PWR0114" H 2600 4450 50  0001 C CNN
F 1 "GND" V 2605 4572 50  0000 R CNN
F 2 "" H 2600 4700 50  0001 C CNN
F 3 "" H 2600 4700 50  0001 C CNN
	1    2600 4700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5F722A39
P 2350 7750
F 0 "#PWR0118" H 2350 7500 50  0001 C CNN
F 1 "GND" H 2355 7577 50  0000 C CNN
F 2 "" H 2350 7750 50  0001 C CNN
F 3 "" H 2350 7750 50  0001 C CNN
	1    2350 7750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 7600 2350 7750
Text Notes 1450 3100 2    50   ~ 0
AD0 has internal pull down
$Comp
L Device:R R6
U 1 1 5F7CFF22
P 1500 6850
F 0 "R6" H 1570 6896 50  0000 L CNN
F 1 "82k" H 1570 6805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1430 6850 50  0001 C CNN
F 3 "~" H 1500 6850 50  0001 C CNN
	1    1500 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5F7D03F7
P 1500 7350
F 0 "R7" H 1570 7396 50  0000 L CNN
F 1 "15k" H 1570 7305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1430 7350 50  0001 C CNN
F 3 "~" H 1500 7350 50  0001 C CNN
	1    1500 7350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5F85250D
P 2350 7150
F 0 "C10" H 2465 7196 50  0000 L CNN
F 1 "100nF" H 2465 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2388 7000 50  0001 C CNN
F 3 "~" H 2350 7150 50  0001 C CNN
	1    2350 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5F85A44C
P 3350 7150
F 0 "C11" H 3465 7196 50  0000 L CNN
F 1 "22uF" H 3465 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3388 7000 50  0001 C CNN
F 3 "~" H 3350 7150 50  0001 C CNN
	1    3350 7150
	1    0    0    -1  
$EndComp
NoConn ~ 1500 2900
NoConn ~ 1500 3000
NoConn ~ 1500 3100
NoConn ~ 2450 950 
NoConn ~ 2450 1050
NoConn ~ 1350 750 
NoConn ~ 2450 1850
Wire Wire Line
	2400 2800 2300 2800
$Comp
L power:GND #PWR0113
U 1 1 5FB0D703
P 2400 2800
F 0 "#PWR0113" H 2400 2550 50  0001 C CNN
F 1 "GND" V 2405 2672 50  0000 R CNN
F 2 "" H 2400 2800 50  0001 C CNN
F 3 "" H 2400 2800 50  0001 C CNN
	1    2400 2800
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR0115
U 1 1 5FB0ED44
P 1200 6500
F 0 "#PWR0115" H 1200 6350 50  0001 C CNN
F 1 "+BATT" H 1215 6673 50  0000 C CNN
F 2 "" H 1200 6500 50  0001 C CNN
F 3 "" H 1200 6500 50  0001 C CNN
	1    1200 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5FB184FC
P 1250 2050
F 0 "#PWR0106" H 1250 1800 50  0001 C CNN
F 1 "GND" V 1255 1922 50  0000 R CNN
F 2 "" H 1250 2050 50  0001 C CNN
F 3 "" H 1250 2050 50  0001 C CNN
	1    1250 2050
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0112
U 1 1 5FB19722
P 1250 2150
F 0 "#PWR0112" H 1250 2000 50  0001 C CNN
F 1 "+5V" V 1265 2278 50  0000 L CNN
F 2 "" H 1250 2150 50  0001 C CNN
F 3 "" H 1250 2150 50  0001 C CNN
	1    1250 2150
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0126
U 1 1 5FB1A25C
P 2550 2150
F 0 "#PWR0126" H 2550 2000 50  0001 C CNN
F 1 "+3.3V" V 2565 2278 50  0000 L CNN
F 2 "" H 2550 2150 50  0001 C CNN
F 3 "" H 2550 2150 50  0001 C CNN
	1    2550 2150
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0122
U 1 1 5FB2B245
P 1450 3900
F 0 "#PWR0122" H 1450 3750 50  0001 C CNN
F 1 "+5V" V 1465 4028 50  0000 L CNN
F 2 "" H 1450 3900 50  0001 C CNN
F 3 "" H 1450 3900 50  0001 C CNN
	1    1450 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5FB4A7A6
P 1450 4000
F 0 "#PWR0102" H 1450 3750 50  0001 C CNN
F 1 "GND" V 1455 3872 50  0000 R CNN
F 2 "" H 1450 4000 50  0001 C CNN
F 3 "" H 1450 4000 50  0001 C CNN
	1    1450 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 4000 1550 4000
$Comp
L power:GND #PWR0107
U 1 1 5FB584EC
P 2550 2050
F 0 "#PWR0107" H 2550 1800 50  0001 C CNN
F 1 "GND" V 2555 1922 50  0000 R CNN
F 2 "" H 2550 2050 50  0001 C CNN
F 3 "" H 2550 2050 50  0001 C CNN
	1    2550 2050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2450 2050 2550 2050
Text Notes 200  750  0    59   ~ 0
EN has internal pull up
Text Notes 1450 3000 2    50   ~ 0
Secondary I2C is not used
$Comp
L power:+BATT #PWR0123
U 1 1 5FB45C3F
P 5250 5600
F 0 "#PWR0123" H 5250 5450 50  0001 C CNN
F 1 "+BATT" H 5265 5773 50  0000 C CNN
F 2 "" H 5250 5600 50  0001 C CNN
F 3 "" H 5250 5600 50  0001 C CNN
	1    5250 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5FB1C3F7
P 5250 7550
F 0 "#PWR0119" H 5250 7300 50  0001 C CNN
F 1 "GND" H 5255 7377 50  0000 C CNN
F 2 "" H 5250 7550 50  0001 C CNN
F 3 "" H 5250 7550 50  0001 C CNN
	1    5250 7550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 7300 6750 7300
Wire Wire Line
	6750 7200 6850 7200
Connection ~ 5250 7350
Wire Wire Line
	5250 7350 5250 7550
$Comp
L Device:CP C9
U 1 1 5F872EAE
P 5450 6000
F 0 "C9" H 5568 6046 50  0000 L CNN
F 1 "100uF, 25V" H 5568 5955 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x5.8" H 5488 5850 50  0001 C CNN
F 3 "~" H 5450 6000 50  0001 C CNN
	1    5450 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5F868C66
P 6250 6000
F 0 "C7" H 6365 6046 50  0000 L CNN
F 1 "100nF" H 6365 5955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6288 5850 50  0001 C CNN
F 3 "~" H 6250 6000 50  0001 C CNN
	1    6250 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 5F769F41
P 6250 6350
F 0 "#PWR0124" H 6250 6100 50  0001 C CNN
F 1 "GND" H 6255 6177 50  0000 C CNN
F 2 "" H 6250 6350 50  0001 C CNN
F 3 "" H 6250 6350 50  0001 C CNN
	1    6250 6350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5F726B92
P 6750 7300
F 0 "#PWR0120" H 6750 7050 50  0001 C CNN
F 1 "GND" V 6755 7172 50  0000 R CNN
F 2 "" H 6750 7300 50  0001 C CNN
F 3 "" H 6750 7300 50  0001 C CNN
	1    6750 7300
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0111
U 1 1 5F6ED7A9
P 6750 7200
F 0 "#PWR0111" H 6750 7050 50  0001 C CNN
F 1 "+3.3V" V 6765 7328 50  0000 L CNN
F 2 "" H 6750 7200 50  0001 C CNN
F 3 "" H 6750 7200 50  0001 C CNN
	1    6750 7200
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 5F6B266B
P 4750 6950
F 0 "#PWR0104" H 4750 6800 50  0001 C CNN
F 1 "+5V" V 4765 7078 50  0000 L CNN
F 2 "" H 4750 6950 50  0001 C CNN
F 3 "" H 4750 6950 50  0001 C CNN
	1    4750 6950
	0    -1   -1   0   
$EndComp
$Comp
L cube-library:MOTOR_ENC M2
U 1 1 5F092189
P 7450 7100
F 0 "M2" H 7728 7146 50  0000 L CNN
F 1 "MOTOR_ENC" H 7728 7055 50  0000 L CNN
F 2 "cube-library:SolderWirePad1x6_p2.54" H 7450 7400 50  0001 C CNN
F 3 "" H 7450 7300 50  0001 C CNN
	1    7450 7100
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:A4950E U2
U 1 1 5EDD061A
P 5250 6850
F 0 "U2" H 5250 7431 50  0000 C CNN
F 1 "A4950E" H 5250 7340 50  0000 C CNN
F 2 "cube-library:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm_HandSolder" H 5250 6300 50  0001 C CNN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A4950-Datasheet.ashx" H 4950 7200 50  0001 C CNN
	1    5250 6850
	1    0    0    -1  
$EndComp
Text Label 6050 7000 0    70   ~ 0
ENCODER2B
Wire Wire Line
	6850 7000 6050 7000
Text Label 6050 6900 0    70   ~ 0
ENCODER2A
Wire Wire Line
	6850 6900 6050 6900
Wire Wire Line
	7450 7550 7450 7500
Wire Wire Line
	5950 7550 5950 6750
Wire Wire Line
	7450 7550 5950 7550
Wire Wire Line
	5650 6750 5950 6750
Wire Wire Line
	7450 6650 7450 6700
Wire Wire Line
	5650 6650 7450 6650
Connection ~ 5250 5750
Connection ~ 5450 5750
Wire Wire Line
	5450 5750 5250 5750
Wire Wire Line
	6250 5750 6250 5850
Wire Wire Line
	5450 5750 6250 5750
Wire Wire Line
	5450 5850 5450 5750
Wire Wire Line
	5250 5750 5250 6450
Wire Wire Line
	5250 5600 5250 5750
Text Label 4350 6750 0    70   ~ 0
MOTOR2B
Wire Wire Line
	4850 6750 4350 6750
Text Label 4350 6650 0    70   ~ 0
MOTOR2A
Wire Wire Line
	4850 6650 4350 6650
Wire Wire Line
	4750 6950 4850 6950
Connection ~ 5350 7350
Wire Wire Line
	5350 7250 5350 7350
Wire Wire Line
	5250 7250 5250 7350
Wire Wire Line
	5350 7350 5250 7350
Wire Wire Line
	5750 7350 5350 7350
Wire Wire Line
	5750 6850 5750 7350
Wire Wire Line
	5650 6850 5750 6850
Connection ~ 6250 6250
Wire Wire Line
	5450 6250 6250 6250
Wire Wire Line
	5450 6150 5450 6250
Wire Wire Line
	6250 6250 6250 6350
Wire Wire Line
	6250 6150 6250 6250
Text Notes 4200 2600 0    59   ~ 0
H bridges:\n\nWe are not using current limiting,\n5V is just tied to VREF because it was close for routing
$Comp
L power:GND #PWR0109
U 1 1 5FB0E3F3
P 5250 4800
F 0 "#PWR0109" H 5250 4550 50  0001 C CNN
F 1 "GND" H 5255 4627 50  0000 C CNN
F 2 "" H 5250 4800 50  0001 C CNN
F 3 "" H 5250 4800 50  0001 C CNN
	1    5250 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5FB0C291
P 6750 4600
F 0 "#PWR0116" H 6750 4350 50  0001 C CNN
F 1 "GND" V 6755 4472 50  0000 R CNN
F 2 "" H 6750 4600 50  0001 C CNN
F 3 "" H 6750 4600 50  0001 C CNN
	1    6750 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	6750 4600 6850 4600
Wire Wire Line
	6750 4500 6850 4500
$Comp
L Device:CP C8
U 1 1 5F86EBBA
P 5450 3300
F 0 "C8" H 5568 3346 50  0000 L CNN
F 1 "100uF, 25V" H 5568 3255 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x5.8" H 5488 3150 50  0001 C CNN
F 3 "~" H 5450 3300 50  0001 C CNN
	1    5450 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5F85D144
P 6250 3300
F 0 "C6" H 6365 3346 50  0000 L CNN
F 1 "100nF" H 6365 3255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6288 3150 50  0001 C CNN
F 3 "~" H 6250 3300 50  0001 C CNN
	1    6250 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4250 4850 4250
$Comp
L power:+BATT #PWR0125
U 1 1 5F783751
P 5250 2900
F 0 "#PWR0125" H 5250 2750 50  0001 C CNN
F 1 "+BATT" H 5265 3073 50  0000 C CNN
F 2 "" H 5250 2900 50  0001 C CNN
F 3 "" H 5250 2900 50  0001 C CNN
	1    5250 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5F7225A2
P 6250 3650
F 0 "#PWR0117" H 6250 3400 50  0001 C CNN
F 1 "GND" H 6255 3477 50  0000 C CNN
F 2 "" H 6250 3650 50  0001 C CNN
F 3 "" H 6250 3650 50  0001 C CNN
	1    6250 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0110
U 1 1 5F6E7E75
P 6750 4500
F 0 "#PWR0110" H 6750 4350 50  0001 C CNN
F 1 "+3.3V" V 6765 4628 50  0000 L CNN
F 2 "" H 6750 4500 50  0001 C CNN
F 3 "" H 6750 4500 50  0001 C CNN
	1    6750 4500
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5F6B001E
P 4750 4250
F 0 "#PWR0103" H 4750 4100 50  0001 C CNN
F 1 "+5V" V 4765 4378 50  0000 L CNN
F 2 "" H 4750 4250 50  0001 C CNN
F 3 "" H 4750 4250 50  0001 C CNN
	1    4750 4250
	0    -1   -1   0   
$EndComp
$Comp
L cube-library:MOTOR_ENC M1
U 1 1 5F08FA5D
P 7450 4400
F 0 "M1" H 7728 4446 50  0000 L CNN
F 1 "MOTOR_ENC" H 7728 4355 50  0000 L CNN
F 2 "cube-library:SolderWirePad1x6_p2.54" H 7450 4700 50  0001 C CNN
F 3 "" H 7450 4600 50  0001 C CNN
	1    7450 4400
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:A4950E U1
U 1 1 5EDCF835
P 5250 4150
F 0 "U1" H 5250 4731 50  0000 C CNN
F 1 "A4950E" H 5250 4640 50  0000 C CNN
F 2 "cube-library:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm_HandSolder" H 5250 3600 50  0001 C CNN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A4950-Datasheet.ashx" H 4950 4500 50  0001 C CNN
	1    5250 4150
	1    0    0    -1  
$EndComp
Text Label 6050 4300 0    70   ~ 0
ENCODER1A
Wire Wire Line
	6850 4300 6050 4300
Text Label 6050 4200 0    70   ~ 0
ENCODER1B
Wire Wire Line
	6850 4200 6050 4200
Wire Wire Line
	7450 3950 7450 4000
Wire Wire Line
	5650 3950 7450 3950
Wire Wire Line
	7450 4850 7450 4800
Wire Wire Line
	5950 4850 7450 4850
Wire Wire Line
	5950 4050 5950 4850
Wire Wire Line
	5650 4050 5950 4050
Wire Wire Line
	5450 3150 5450 3050
Wire Wire Line
	5250 3750 5250 3050
Wire Wire Line
	5250 2900 5250 3050
Connection ~ 5250 3050
Wire Wire Line
	5450 3050 5250 3050
Connection ~ 5450 3050
Wire Wire Line
	6250 3050 5450 3050
Wire Wire Line
	6250 3150 6250 3050
Text Label 4350 4050 0    70   ~ 0
MOTOR1A
Wire Wire Line
	4850 4050 4350 4050
Text Label 4350 3950 0    70   ~ 0
MOTOR1B
Wire Wire Line
	4850 3950 4350 3950
Wire Wire Line
	5250 4650 5250 4800
Wire Wire Line
	5250 4650 5250 4550
Connection ~ 5250 4650
Wire Wire Line
	5350 4650 5250 4650
Wire Wire Line
	5350 4550 5350 4650
Connection ~ 5350 4650
Wire Wire Line
	5750 4650 5350 4650
Wire Wire Line
	5650 4150 5750 4150
Wire Wire Line
	5750 4150 5750 4650
Wire Wire Line
	5450 3550 6250 3550
Wire Wire Line
	5450 3450 5450 3550
Wire Wire Line
	6250 3550 6250 3450
Connection ~ 6250 3550
Wire Wire Line
	6250 3650 6250 3550
Text Notes 8200 6300 0    59   ~ 0
Line sensor LEDs\n\nCharlieplexing leds, using ESP32 GPIO drive strength to limit current.\nShould be safe even at max drive strength at about 30mA per LED.\n\nSee https://www.esp32.com/viewtopic.php?f=12&t=5840#p25550
Text Notes 7850 500  0    59   ~ 0
Line sensor
Text GLabel 9050 7650 0    50   Input ~ 0
LINE_LED2
Text GLabel 9050 7150 0    50   Input ~ 0
LINE_LED1
Text GLabel 9050 6650 0    50   Input ~ 0
LINE_LED0
Wire Wire Line
	10100 7550 10100 7650
Connection ~ 10100 7650
Wire Wire Line
	9700 7650 10100 7650
Wire Wire Line
	9700 7550 9700 7650
Connection ~ 9700 7650
Wire Wire Line
	9050 7650 9700 7650
Wire Wire Line
	10900 7350 10900 7650
Wire Wire Line
	10100 7650 10500 7650
Wire Wire Line
	10500 7650 10900 7650
Connection ~ 10500 7650
Wire Wire Line
	10500 7350 10500 7650
Wire Wire Line
	10900 6650 10900 7050
Wire Wire Line
	10500 6650 10900 6650
Wire Wire Line
	10500 7050 10500 6650
Connection ~ 10500 6650
Wire Wire Line
	10100 6650 10500 6650
Wire Wire Line
	10100 6750 10100 6650
Wire Wire Line
	10100 7250 10100 7150
Wire Wire Line
	10100 7050 10100 7150
Connection ~ 10100 7150
Wire Wire Line
	9700 7150 10100 7150
Connection ~ 10100 6650
Wire Wire Line
	9700 6650 10100 6650
Wire Wire Line
	9700 6750 9700 6650
Connection ~ 9700 6650
Wire Wire Line
	9050 6650 9700 6650
$Comp
L Device:LED D4
U 1 1 5F6A8D59
P 10900 7200
F 0 "D4" V 10939 7082 50  0000 R CNN
F 1 "APT2012EC" V 10848 7082 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10900 7200 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 10900 7200 50  0001 C CNN
	1    10900 7200
	0    1    1    0   
$EndComp
$Comp
L Device:LED D3
U 1 1 5F6A8692
P 10500 7200
F 0 "D3" V 10447 7279 50  0000 L CNN
F 1 "APT2012EC" V 10538 7279 50  0000 L CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10500 7200 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 10500 7200 50  0001 C CNN
	1    10500 7200
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D6
U 1 1 5F6A2E93
P 10100 7400
F 0 "D6" V 10047 7479 50  0000 L CNN
F 1 "APT2012EC" V 10138 7479 50  0000 L CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10100 7400 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 10100 7400 50  0001 C CNN
	1    10100 7400
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 5F6A2946
P 9700 7400
F 0 "D5" V 9739 7282 50  0000 R CNN
F 1 "APT2012EC" V 9648 7282 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9700 7400 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 9700 7400 50  0001 C CNN
	1    9700 7400
	0    1    1    0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5F69D55B
P 10100 6900
F 0 "D2" V 10139 6782 50  0000 R CNN
F 1 "APT2012EC" V 10048 6782 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10100 6900 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 10100 6900 50  0001 C CNN
	1    10100 6900
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5F69B5F1
P 9700 6900
F 0 "D1" V 9647 6979 50  0000 L CNN
F 1 "APT2012EC" V 9738 6979 50  0000 L CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9700 6900 50  0001 C CNN
F 3 "http://www.kingbrightusa.com/images/catalog/SPEC/APT2012EC.pdf" H 9700 6900 50  0001 C CNN
	1    9700 6900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9700 7150 9700 7250
Wire Wire Line
	9700 7050 9700 7150
Connection ~ 9700 7150
Wire Wire Line
	9050 7150 9700 7150
Text GLabel 10400 4000 2    50   Output ~ 0
LINE_SENSOR4
Text GLabel 10400 3200 2    50   Output ~ 0
LINE_SENSOR3
Text GLabel 10400 2400 2    50   Output ~ 0
LINE_SENSOR2
Text GLabel 10400 1600 2    50   Output ~ 0
LINE_SENSOR1
Text GLabel 10400 800  2    50   Output ~ 0
LINE_SENSOR0
$Comp
L Device:Q_Photo_NPN Q5
U 1 1 5F8D10CE
P 8950 4100
F 0 "Q5" V 8635 4100 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8726 4100 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9150 4200 50  0001 C CNN
F 3 "~" H 8950 4100 50  0001 C CNN
	1    8950 4100
	0    -1   1    0   
$EndComp
$Comp
L Device:Q_Photo_NPN Q4
U 1 1 5F8D075D
P 8950 3300
F 0 "Q4" V 8635 3300 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8726 3300 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9150 3400 50  0001 C CNN
F 3 "~" H 8950 3300 50  0001 C CNN
	1    8950 3300
	0    -1   1    0   
$EndComp
$Comp
L Device:Q_Photo_NPN Q3
U 1 1 5F8D019A
P 8950 2500
F 0 "Q3" V 8635 2500 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8726 2500 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9150 2600 50  0001 C CNN
F 3 "~" H 8950 2500 50  0001 C CNN
	1    8950 2500
	0    -1   1    0   
$EndComp
$Comp
L Device:Q_Photo_NPN Q2
U 1 1 5F8CEDFE
P 8950 1700
F 0 "Q2" V 8635 1700 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8726 1700 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9150 1800 50  0001 C CNN
F 3 "~" H 8950 1700 50  0001 C CNN
	1    8950 1700
	0    -1   1    0   
$EndComp
$Comp
L Device:Q_Photo_NPN Q1
U 1 1 5F883D85
P 8950 900
F 0 "Q1" V 8635 900 50  0000 C CNN
F 1 "ALS-PT19-315C_L177_TR8" V 8726 900 50  0000 C CNN
F 2 "cube-library:phototransistor-1.7mm" H 9150 1000 50  0001 C CNN
F 3 "~" H 8950 900 50  0001 C CNN
	1    8950 900 
	0    -1   1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 5F8649E1
P 9700 4400
F 0 "C5" V 9448 4400 50  0000 C CNN
F 1 "100nF" V 9539 4400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 4250 50  0001 C CNN
F 3 "~" H 9700 4400 50  0001 C CNN
	1    9700 4400
	0    1    1    0   
$EndComp
$Comp
L Device:C C4
U 1 1 5F864455
P 9700 3600
F 0 "C4" V 9448 3600 50  0000 C CNN
F 1 "100nF" V 9539 3600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 3450 50  0001 C CNN
F 3 "~" H 9700 3600 50  0001 C CNN
	1    9700 3600
	0    1    1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 5F863E50
P 9700 2800
F 0 "C3" V 9448 2800 50  0000 C CNN
F 1 "100nF" V 9539 2800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 2650 50  0001 C CNN
F 3 "~" H 9700 2800 50  0001 C CNN
	1    9700 2800
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5F863769
P 9700 2000
F 0 "C2" V 9448 2000 50  0000 C CNN
F 1 "100nF" V 9539 2000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 1850 50  0001 C CNN
F 3 "~" H 9700 2000 50  0001 C CNN
	1    9700 2000
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 5F862D5E
P 9700 1200
F 0 "C1" V 9448 1200 50  0000 C CNN
F 1 "100nF" V 9539 1200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 1050 50  0001 C CNN
F 3 "~" H 9700 1200 50  0001 C CNN
	1    9700 1200
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5F810985
P 9700 4200
F 0 "R5" V 9493 4200 50  0000 C CNN
F 1 "5k6" V 9584 4200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9630 4200 50  0001 C CNN
F 3 "~" H 9700 4200 50  0001 C CNN
	1    9700 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5F810521
P 9700 3400
F 0 "R4" V 9493 3400 50  0000 C CNN
F 1 "5k6" V 9584 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9630 3400 50  0001 C CNN
F 3 "~" H 9700 3400 50  0001 C CNN
	1    9700 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5F8100EF
P 9700 2600
F 0 "R3" V 9493 2600 50  0000 C CNN
F 1 "5k6" V 9584 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9630 2600 50  0001 C CNN
F 3 "~" H 9700 2600 50  0001 C CNN
	1    9700 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5F80FD5B
P 9700 1800
F 0 "R2" V 9493 1800 50  0000 C CNN
F 1 "5k6" V 9584 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9630 1800 50  0001 C CNN
F 3 "~" H 9700 1800 50  0001 C CNN
	1    9700 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5F7F32DB
P 9700 1000
F 0 "R1" V 9493 1000 50  0000 C CNN
F 1 "5k6" V 9584 1000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9630 1000 50  0001 C CNN
F 3 "~" H 9700 1000 50  0001 C CNN
	1    9700 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5F72ACF9
P 10050 4500
F 0 "#PWR0121" H 10050 4250 50  0001 C CNN
F 1 "GND" H 10055 4327 50  0000 C CNN
F 2 "" H 10050 4500 50  0001 C CNN
F 3 "" H 10050 4500 50  0001 C CNN
	1    10050 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5F6B9B15
P 8450 1000
F 0 "#PWR0108" H 8450 850 50  0001 C CNN
F 1 "+3.3V" V 8465 1128 50  0000 L CNN
F 2 "" H 8450 1000 50  0001 C CNN
F 3 "" H 8450 1000 50  0001 C CNN
	1    8450 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9150 1000 9350 1000
Wire Wire Line
	9350 800  10400 800 
Wire Wire Line
	9350 1000 9350 800 
Connection ~ 9350 1000
Wire Wire Line
	9350 1200 9350 1000
Wire Wire Line
	9550 1200 9350 1200
Wire Wire Line
	9350 1000 9550 1000
Wire Wire Line
	9150 1800 9350 1800
Wire Wire Line
	9350 1600 10400 1600
Wire Wire Line
	9350 1800 9350 1600
Connection ~ 9350 1800
Wire Wire Line
	9350 2000 9350 1800
Wire Wire Line
	9550 2000 9350 2000
Wire Wire Line
	9350 1800 9550 1800
Wire Wire Line
	9150 2600 9350 2600
Wire Wire Line
	9350 2400 10400 2400
Wire Wire Line
	9350 2600 9350 2400
Connection ~ 9350 2600
Wire Wire Line
	9350 2800 9350 2600
Wire Wire Line
	9550 2800 9350 2800
Wire Wire Line
	9350 2600 9550 2600
Wire Wire Line
	9350 3400 9150 3400
Wire Wire Line
	9350 3200 10400 3200
Wire Wire Line
	9350 3400 9350 3200
Connection ~ 9350 3400
Wire Wire Line
	9350 3600 9350 3400
Wire Wire Line
	9550 3600 9350 3600
Wire Wire Line
	9350 3400 9550 3400
Wire Wire Line
	9350 4200 9150 4200
Wire Wire Line
	9350 4000 10400 4000
Wire Wire Line
	9350 4200 9350 4000
Wire Wire Line
	9350 4400 9550 4400
Connection ~ 9350 4200
Wire Wire Line
	9350 4200 9350 4400
Wire Wire Line
	9550 4200 9350 4200
Wire Wire Line
	8750 2600 8650 2600
Wire Wire Line
	8750 4200 8650 4200
Wire Wire Line
	8750 3400 8650 3400
Wire Wire Line
	8750 1800 8650 1800
Wire Wire Line
	8650 1000 8750 1000
Wire Wire Line
	8450 1000 8650 1000
Connection ~ 8650 1000
Wire Wire Line
	8650 1800 8650 1000
Connection ~ 8650 1800
Wire Wire Line
	8650 2600 8650 1800
Connection ~ 8650 2600
Wire Wire Line
	8650 3400 8650 2600
Connection ~ 8650 3400
Wire Wire Line
	8650 4200 8650 3400
Wire Wire Line
	10050 1000 9850 1000
Wire Wire Line
	10050 1000 10050 1200
Connection ~ 10050 1200
Wire Wire Line
	9850 1200 10050 1200
Wire Wire Line
	10050 1800 10050 1200
Connection ~ 10050 1800
Wire Wire Line
	10050 1800 9850 1800
Wire Wire Line
	10050 2000 10050 1800
Connection ~ 10050 2000
Wire Wire Line
	9850 2000 10050 2000
Wire Wire Line
	10050 2600 10050 2000
Connection ~ 10050 2600
Wire Wire Line
	10050 2600 9850 2600
Wire Wire Line
	10050 2800 10050 2600
Connection ~ 10050 2800
Wire Wire Line
	9850 2800 10050 2800
Wire Wire Line
	10050 3400 10050 2800
Connection ~ 10050 3400
Wire Wire Line
	10050 3400 9850 3400
Wire Wire Line
	10050 3600 10050 3400
Connection ~ 10050 3600
Wire Wire Line
	9850 3600 10050 3600
Wire Wire Line
	10050 4500 10050 4400
Connection ~ 10050 4400
Wire Wire Line
	10050 4400 9850 4400
Wire Wire Line
	10050 4200 10050 3600
Wire Wire Line
	10050 4200 10050 4400
Connection ~ 10050 4200
Wire Wire Line
	9850 4200 10050 4200
$EndSCHEMATC
