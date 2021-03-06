EESchema Schematic File Version 4
LIBS:LED suit power-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
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
L 74hct245:74HCT245 U?
U 1 1 5BB68D0F
P 4750 3700
AR Path="/5BB68D0F" Ref="U?"  Part="1" 
AR Path="/5BB67CCC/5BB68D0F" Ref="U8"  Part="1" 
F 0 "U8" H 4750 4487 60  0000 C CNN
F 1 "74HCT245" H 4750 4381 60  0000 C CNN
F 2 "sop:TSSOP20" H 4750 3000 60  0001 C CNN
F 3 "" H 4750 3700 60  0000 C CNN
	1    4750 3700
	1    0    0    -1  
$EndComp
$Comp
L Connector:RJ45 J?
U 1 1 5BB68D16
P 7250 3200
AR Path="/5BB68D16" Ref="J?"  Part="1" 
AR Path="/5BB67CCC/5BB68D16" Ref="J8"  Part="1" 
F 0 "J8" H 6920 3296 50  0000 R CNN
F 1 "RJ45" H 6920 3205 50  0000 R CNN
F 2 "Connectors:RJ45_8" V 7250 3225 50  0001 C CNN
F 3 "~" V 7250 3225 50  0001 C CNN
	1    7250 3200
	-1   0    0    -1  
$EndComp
$Comp
L Connector:RJ45 J?
U 1 1 5BB68D40
P 7250 4200
AR Path="/5BB68D40" Ref="J?"  Part="1" 
AR Path="/5BB67CCC/5BB68D40" Ref="J9"  Part="1" 
F 0 "J9" H 6920 4296 50  0000 R CNN
F 1 "RJ45" H 6920 4205 50  0000 R CNN
F 2 "Connectors:RJ45_8" V 7250 4225 50  0001 C CNN
F 3 "~" V 7250 4225 50  0001 C CNN
	1    7250 4200
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5BB68EDB
P 6250 2800
F 0 "R1" V 6054 2800 50  0000 C CNN
F 1 "R_Small" V 6145 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6250 2800 50  0001 C CNN
F 3 "~" H 6250 2800 50  0001 C CNN
	1    6250 2800
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5BB68F40
P 6250 3000
F 0 "R2" V 6054 3000 50  0000 C CNN
F 1 "R_Small" V 6145 3000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6250 3000 50  0001 C CNN
F 3 "~" H 6250 3000 50  0001 C CNN
	1    6250 3000
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5BB69009
P 6250 3200
F 0 "R3" V 6054 3200 50  0000 C CNN
F 1 "R_Small" V 6145 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6250 3200 50  0001 C CNN
F 3 "~" H 6250 3200 50  0001 C CNN
	1    6250 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5BB69010
P 6250 3400
F 0 "R4" V 6054 3400 50  0000 C CNN
F 1 "R_Small" V 6145 3400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6250 3400 50  0001 C CNN
F 3 "~" H 6250 3400 50  0001 C CNN
	1    6250 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5BB69081
P 6250 3800
F 0 "R5" V 6054 3800 50  0000 C CNN
F 1 "R_Small" V 6145 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6250 3800 50  0001 C CNN
F 3 "~" H 6250 3800 50  0001 C CNN
	1    6250 3800
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5BB69088
P 6250 4000
F 0 "R6" V 6054 4000 50  0000 C CNN
F 1 "R_Small" V 6145 4000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6250 4000 50  0001 C CNN
F 3 "~" H 6250 4000 50  0001 C CNN
	1    6250 4000
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5BB6908F
P 6250 4200
F 0 "R7" V 6054 4200 50  0000 C CNN
F 1 "R_Small" V 6145 4200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6250 4200 50  0001 C CNN
F 3 "~" H 6250 4200 50  0001 C CNN
	1    6250 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R8
U 1 1 5BB69096
P 6250 4400
F 0 "R8" V 6054 4400 50  0000 C CNN
F 1 "R_Small" V 6145 4400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6250 4400 50  0001 C CNN
F 3 "~" H 6250 4400 50  0001 C CNN
	1    6250 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 2900 6700 2900
Wire Wire Line
	6700 2900 6700 3100
Wire Wire Line
	6850 3100 6700 3100
Connection ~ 6700 3100
Wire Wire Line
	6700 3100 6700 3300
Wire Wire Line
	6850 3300 6700 3300
Connection ~ 6700 3300
Wire Wire Line
	6850 3900 6700 3900
Connection ~ 6700 3900
Wire Wire Line
	6700 3900 6700 4100
Wire Wire Line
	6850 4100 6700 4100
Connection ~ 6700 4100
Wire Wire Line
	6700 4100 6700 4300
Wire Wire Line
	6850 4300 6700 4300
Connection ~ 6700 4300
Wire Wire Line
	6700 4300 6700 4500
Wire Wire Line
	6350 2800 6850 2800
Wire Wire Line
	6850 3000 6350 3000
Wire Wire Line
	6350 3200 6850 3200
Wire Wire Line
	6850 3400 6350 3400
Wire Wire Line
	6350 3800 6850 3800
Wire Wire Line
	6850 4000 6350 4000
Wire Wire Line
	6350 4200 6850 4200
Wire Wire Line
	6850 4400 6350 4400
Wire Wire Line
	6150 4400 5750 4400
Wire Wire Line
	5750 4400 5750 3900
Wire Wire Line
	5750 3900 5450 3900
Wire Wire Line
	5450 3800 5850 3800
Wire Wire Line
	5850 3800 5850 4200
Wire Wire Line
	5850 4200 6150 4200
Wire Wire Line
	6150 4000 5950 4000
Wire Wire Line
	5950 4000 5950 3700
Wire Wire Line
	5950 3700 5450 3700
Wire Wire Line
	5450 3600 6050 3600
Wire Wire Line
	6050 3600 6050 3800
Wire Wire Line
	6050 3800 6150 3800
Wire Wire Line
	6150 3400 6050 3400
Wire Wire Line
	6050 3400 6050 3500
Wire Wire Line
	6050 3500 5450 3500
Wire Wire Line
	5450 3400 5950 3400
Wire Wire Line
	5950 3400 5950 3200
Wire Wire Line
	5950 3200 6150 3200
Wire Wire Line
	6150 3000 5850 3000
Wire Wire Line
	5850 3000 5850 3300
Wire Wire Line
	5850 3300 5450 3300
Wire Wire Line
	5450 3200 5750 3200
Wire Wire Line
	5750 3200 5750 2800
Wire Wire Line
	5750 2800 6150 2800
$Comp
L power:GND #PWR0114
U 1 1 5BB6EACD
P 6700 4700
F 0 "#PWR0114" H 6700 4450 50  0001 C CNN
F 1 "GND" H 6705 4527 50  0000 C CNN
F 2 "" H 6700 4700 50  0001 C CNN
F 3 "" H 6700 4700 50  0001 C CNN
	1    6700 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5BB6EB17
P 5500 4250
F 0 "#PWR0115" H 5500 4000 50  0001 C CNN
F 1 "GND" H 5505 4077 50  0000 C CNN
F 2 "" H 5500 4250 50  0001 C CNN
F 3 "" H 5500 4250 50  0001 C CNN
	1    5500 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 4250 5500 4150
Wire Wire Line
	5500 4150 5450 4150
$Comp
L power:+5V #PWR0116
U 1 1 5BB6FAAC
P 5550 2500
F 0 "#PWR0116" H 5550 2350 50  0001 C CNN
F 1 "+5V" H 5565 2673 50  0000 C CNN
F 2 "" H 5550 2500 50  0001 C CNN
F 3 "" H 5550 2500 50  0001 C CNN
	1    5550 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2500 5550 2800
Wire Wire Line
	5550 4050 5450 4050
Wire Wire Line
	4050 4100 3800 4100
Wire Wire Line
	3800 4100 3800 2800
Wire Wire Line
	3800 2800 5550 2800
Connection ~ 5550 2800
Wire Wire Line
	5550 2800 5550 4050
Wire Wire Line
	4050 4200 3800 4200
Wire Wire Line
	3800 4200 3800 4350
$Comp
L power:GND #PWR0117
U 1 1 5BB72ED1
P 3800 4350
F 0 "#PWR0117" H 3800 4100 50  0001 C CNN
F 1 "GND" H 3805 4177 50  0000 C CNN
F 2 "" H 3800 4350 50  0001 C CNN
F 3 "" H 3800 4350 50  0001 C CNN
	1    3800 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5BB74424
P 4350 2250
F 0 "C7" H 4442 2296 50  0000 L CNN
F 1 "C_Small" H 4442 2205 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4350 2250 50  0001 C CNN
F 3 "~" H 4350 2250 50  0001 C CNN
	1    4350 2250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0118
U 1 1 5BB7448C
P 4350 2150
F 0 "#PWR0118" H 4350 2000 50  0001 C CNN
F 1 "+5V" H 4365 2323 50  0000 C CNN
F 2 "" H 4350 2150 50  0001 C CNN
F 3 "" H 4350 2150 50  0001 C CNN
	1    4350 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5BB75786
P 4350 2350
F 0 "#PWR0119" H 4350 2100 50  0001 C CNN
F 1 "GND" H 4355 2177 50  0000 C CNN
F 2 "" H 4350 2350 50  0001 C CNN
F 3 "" H 4350 2350 50  0001 C CNN
	1    4350 2350
	1    0    0    -1  
$EndComp
Text GLabel 4050 3200 0    50   Input ~ 0
PIN2
Text GLabel 4050 3300 0    50   Input ~ 0
PIN14
Text GLabel 4050 3400 0    50   Input ~ 0
PIN7
Text GLabel 4050 3500 0    50   Input ~ 0
PIN8
Text GLabel 4050 3600 0    50   Input ~ 0
PIN6
Text GLabel 4050 3700 0    50   Input ~ 0
PIN20
Text GLabel 4050 3800 0    50   Input ~ 0
PIN21
Text GLabel 4050 3900 0    50   Input ~ 0
PIN5
Wire Wire Line
	6700 3300 6700 3500
Wire Wire Line
	6850 4500 6700 4500
Connection ~ 6700 4500
Wire Wire Line
	6700 4500 6700 4700
Wire Wire Line
	6700 3500 6850 3500
Connection ~ 6700 3500
Wire Wire Line
	6700 3500 6700 3900
$EndSCHEMATC
