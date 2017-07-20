EESchema Schematic File Version 2
LIBS:MAGLabs-2017-Swadges-rescue
LIBS:power
LIBS:device
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
LIBS:local
LIBS:MAGLabs-2017-Swadges-cache
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
L ESP12E U1
U 1 1 596EC18A
P 3350 2050
F 0 "U1" H 3300 2787 60  0000 C CNN
F 1 "ESP12E" H 3300 2681 60  0000 C CNN
F 2 "local:ESP12E" H 3350 2000 60  0001 C CNN
F 3 "" H 3350 2000 60  0001 C CNN
	1    3350 2050
	1    0    0    -1  
$EndComp
$Comp
L Tactile_Switch-RESCUE-MAGLabs-2017-Swadges S1
U 1 1 596EC645
P 6150 950
F 0 "S1" V 6203 1009 60  0000 R CNN
F 1 "Up" V 6300 1250 60  0000 R CNN
F 2 "local:PTS645" H 6350 950 60  0001 C CNN
F 3 "" H 6350 950 60  0001 C CNN
	1    6150 950 
	0    -1   -1   0   
$EndComp
$Comp
L Tactile_Switch-RESCUE-MAGLabs-2017-Swadges S2
U 1 1 596EC831
P 6600 950
F 0 "S2" V 6653 1009 60  0000 R CNN
F 1 "Down" V 6750 1300 60  0000 R CNN
F 2 "local:PTS645" H 6800 950 60  0001 C CNN
F 3 "" H 6800 950 60  0001 C CNN
	1    6600 950 
	0    -1   -1   0   
$EndComp
$Comp
L Tactile_Switch-RESCUE-MAGLabs-2017-Swadges S3
U 1 1 596EC875
P 6150 2250
F 0 "S3" V 6203 2309 60  0000 R CNN
F 1 "Left" V 6300 2600 60  0000 R CNN
F 2 "local:PTS645" H 6350 2250 60  0001 C CNN
F 3 "" H 6350 2250 60  0001 C CNN
	1    6150 2250
	0    -1   -1   0   
$EndComp
$Comp
L Tactile_Switch-RESCUE-MAGLabs-2017-Swadges S4
U 1 1 596EC8B5
P 6600 2250
F 0 "S4" V 6653 2309 60  0000 R CNN
F 1 "Right" V 6750 2600 60  0000 R CNN
F 2 "local:PTS645" H 6800 2250 60  0001 C CNN
F 3 "" H 6800 2250 60  0001 C CNN
	1    6600 2250
	0    -1   -1   0   
$EndComp
$Comp
L Tactile_Switch-RESCUE-MAGLabs-2017-Swadges S5
U 1 1 596EC995
P 7100 1600
F 0 "S5" V 7153 1659 60  0000 R CNN
F 1 "A" V 7250 1850 60  0000 R CNN
F 2 "local:PTS645" H 7300 1600 60  0001 C CNN
F 3 "" H 7300 1600 60  0001 C CNN
	1    7100 1600
	0    -1   -1   0   
$EndComp
$Comp
L Tactile_Switch-RESCUE-MAGLabs-2017-Swadges S6
U 1 1 596EC9D7
P 7550 1600
F 0 "S6" V 7603 1659 60  0000 R CNN
F 1 "B" V 7700 1850 60  0000 R CNN
F 2 "local:PTS645" H 7750 1600 60  0001 C CNN
F 3 "" H 7750 1600 60  0001 C CNN
	1    7550 1600
	0    -1   -1   0   
$EndComp
$Comp
L Tactile_Switch-RESCUE-MAGLabs-2017-Swadges S7
U 1 1 596ECA1D
P 9900 1750
F 0 "S7" V 9953 1809 60  0000 R CNN
F 1 "Start/Program" H 10250 2250 60  0000 R CNN
F 2 "local:PTS645" H 10100 1750 60  0001 C CNN
F 3 "" H 10100 1750 60  0001 C CNN
	1    9900 1750
	0    -1   -1   0   
$EndComp
$Comp
L Tactile_Switch-RESCUE-MAGLabs-2017-Swadges S8
U 1 1 596ECA67
P 10450 1750
F 0 "S8" V 10503 1809 60  0000 R CNN
F 1 "Select" H 10600 2150 60  0000 R CNN
F 2 "local:PTS645" H 10650 1750 60  0001 C CNN
F 3 "" H 10650 1750 60  0001 C CNN
	1    10450 1750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5300 2800 9550 2800
Wire Wire Line
	5200 2900 10100 2900
$Comp
L GND #PWR01
U 1 1 596EFDB7
P 9800 2100
F 0 "#PWR01" H 9800 1850 50  0001 C CNN
F 1 "GND" H 9805 1927 50  0000 C CNN
F 2 "" H 9800 2100 50  0001 C CNN
F 3 "" H 9800 2100 50  0001 C CNN
	1    9800 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 596EFDE1
P 10350 2100
F 0 "#PWR02" H 10350 1850 50  0001 C CNN
F 1 "GND" H 10355 1927 50  0000 C CNN
F 2 "" H 10350 2100 50  0001 C CNN
F 3 "" H 10350 2100 50  0001 C CNN
	1    10350 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 596F011E
P 4200 2400
F 0 "#PWR03" H 4200 2150 50  0001 C CNN
F 1 "GND" H 4205 2227 50  0000 C CNN
F 2 "" H 4200 2400 50  0001 C CNN
F 3 "" H 4200 2400 50  0001 C CNN
	1    4200 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 2400 4200 2350
Wire Wire Line
	4200 2350 4100 2350
$Comp
L +3.3V #PWR04
U 1 1 596F029A
P 2500 2350
F 0 "#PWR04" H 2500 2200 50  0001 C CNN
F 1 "+3.3V" V 2515 2478 50  0000 L CNN
F 2 "" H 2500 2350 50  0001 C CNN
F 3 "" H 2500 2350 50  0001 C CNN
	1    2500 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5300 2800 5300 2050
Wire Wire Line
	5300 2050 4100 2050
Wire Wire Line
	5200 2900 5200 2150
Wire Wire Line
	5200 2150 4100 2150
Wire Wire Line
	10100 2900 10100 2100
Wire Wire Line
	9550 2800 9550 2100
$Comp
L R R2
U 1 1 596F08C9
P 9550 1250
F 0 "R2" H 9620 1296 50  0000 L CNN
F 1 "10k" H 9620 1205 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 9480 1250 50  0001 C CNN
F 3 "" H 9550 1250 50  0001 C CNN
	1    9550 1250
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 596F092F
P 10100 1250
F 0 "R3" H 10170 1296 50  0000 L CNN
F 1 "10k" H 10170 1205 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 10030 1250 50  0001 C CNN
F 3 "" H 10100 1250 50  0001 C CNN
	1    10100 1250
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 596F0986
P 9550 1100
F 0 "#PWR05" H 9550 950 50  0001 C CNN
F 1 "+3.3V" H 9565 1273 50  0000 C CNN
F 2 "" H 9550 1100 50  0001 C CNN
F 3 "" H 9550 1100 50  0001 C CNN
	1    9550 1100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR06
U 1 1 596F09B4
P 10100 1100
F 0 "#PWR06" H 10100 950 50  0001 C CNN
F 1 "+3.3V" H 10115 1273 50  0000 C CNN
F 2 "" H 10100 1100 50  0001 C CNN
F 3 "" H 10100 1100 50  0001 C CNN
	1    10100 1100
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 596F0EF8
P 2400 1450
F 0 "R1" H 2470 1496 50  0000 L CNN
F 1 "10k" H 2470 1405 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 2330 1450 50  0001 C CNN
F 3 "" H 2400 1450 50  0001 C CNN
	1    2400 1450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 596F0F70
P 2400 1300
F 0 "#PWR07" H 2400 1150 50  0001 C CNN
F 1 "+3.3V" H 2415 1473 50  0000 C CNN
F 2 "" H 2400 1300 50  0001 C CNN
F 3 "" H 2400 1300 50  0001 C CNN
	1    2400 1300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR08
U 1 1 596F0FA0
P 2500 1850
F 0 "#PWR08" H 2500 1700 50  0001 C CNN
F 1 "+3.3V" V 2515 1978 50  0000 L CNN
F 2 "" H 2500 1850 50  0001 C CNN
F 3 "" H 2500 1850 50  0001 C CNN
	1    2500 1850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2500 1650 2400 1650
Wire Wire Line
	2400 1650 2400 1600
$Comp
L CONN_01X03 J2
U 1 1 596F104F
P 2450 3350
F 0 "J2" H 2527 3391 50  0000 L CNN
F 1 "Valve Magic" H 2527 3300 50  0000 L CNN
F 2 "VALVE_CHICKLET_8_1" H 2450 3350 50  0001 C CNN
F 3 "" H 2450 3350 50  0001 C CNN
	1    2450 3350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR09
U 1 1 596F10C2
P 2150 3200
F 0 "#PWR09" H 2150 3050 50  0001 C CNN
F 1 "+3.3V" H 2165 3373 50  0000 C CNN
F 2 "" H 2150 3200 50  0001 C CNN
F 3 "" H 2150 3200 50  0001 C CNN
	1    2150 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 596F110D
P 2150 3350
F 0 "#PWR010" H 2150 3100 50  0001 C CNN
F 1 "GND" H 2155 3177 50  0000 C CNN
F 2 "" H 2150 3350 50  0001 C CNN
F 3 "" H 2150 3350 50  0001 C CNN
	1    2150 3350
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 3250 2150 3250
Wire Wire Line
	2150 3250 2150 3200
Wire Wire Line
	2250 3350 2150 3350
$Comp
L +3.3V #PWR011
U 1 1 596F137E
P 1150 1600
F 0 "#PWR011" H 1150 1450 50  0001 C CNN
F 1 "+3.3V" H 1165 1773 50  0000 C CNN
F 2 "" H 1150 1600 50  0001 C CNN
F 3 "" H 1150 1600 50  0001 C CNN
	1    1150 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 596F13CC
P 1150 2200
F 0 "#PWR012" H 1150 1950 50  0001 C CNN
F 1 "GND" H 1155 2027 50  0000 C CNN
F 2 "" H 1150 2200 50  0001 C CNN
F 3 "" H 1150 2200 50  0001 C CNN
	1    1150 2200
	1    0    0    -1  
$EndComp
NoConn ~ 3550 2700
NoConn ~ 3450 2700
NoConn ~ 3350 2700
NoConn ~ 3250 2700
NoConn ~ 3150 2700
NoConn ~ 3050 2700
Wire Wire Line
	1050 1650 1150 1650
Wire Wire Line
	1150 1650 1150 1600
Wire Wire Line
	1050 2150 1150 2150
Wire Wire Line
	1150 2150 1150 2200
Wire Wire Line
	1250 2050 1250 2250
Wire Wire Line
	1250 2050 1050 2050
Wire Wire Line
	1350 1950 1050 1950
Wire Wire Line
	1050 1850 1450 1850
$Comp
L WS2812B D4
U 1 1 596F21C8
P 9150 3800
F 0 "D4" H 9125 4337 60  0000 C CNN
F 1 "WS2812B" H 9125 4231 60  0000 C CNN
F 2 "local:WS2812B" H 9150 3450 60  0001 C CNN
F 3 "" H 9150 3450 60  0001 C CNN
	1    9150 3800
	1    0    0    -1  
$EndComp
$Comp
L WS2812B D5
U 1 1 596F2238
P 9150 4500
F 0 "D5" H 9125 5037 60  0000 C CNN
F 1 "WS2812B" H 9125 4931 60  0000 C CNN
F 2 "local:WS2812B" H 9150 4150 60  0001 C CNN
F 3 "" H 9150 4150 60  0001 C CNN
	1    9150 4500
	1    0    0    -1  
$EndComp
$Comp
L WS2812B D6
U 1 1 596F2371
P 9150 5200
F 0 "D6" H 9125 5737 60  0000 C CNN
F 1 "WS2812B" H 9125 5631 60  0000 C CNN
F 2 "local:WS2812B" H 9150 4850 60  0001 C CNN
F 3 "" H 9150 4850 60  0001 C CNN
	1    9150 5200
	1    0    0    -1  
$EndComp
$Comp
L WS2812B D7
U 1 1 596F23C7
P 9150 5900
F 0 "D7" H 9125 6437 60  0000 C CNN
F 1 "WS2812B" H 9125 6331 60  0000 C CNN
F 2 "local:WS2812B" H 9150 5550 60  0001 C CNN
F 3 "" H 9150 5550 60  0001 C CNN
	1    9150 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 3550 9850 3550
Text GLabel 4100 1750 2    60   Input ~ 0
RXD
Text GLabel 4100 1650 2    60   Input ~ 0
TXD
Text GLabel 9850 3550 2    60   Input ~ 0
RXD
Wire Wire Line
	8650 3700 8600 3700
Wire Wire Line
	8600 3700 8600 3850
Wire Wire Line
	8600 3850 9650 3850
Wire Wire Line
	9650 3850 9650 4250
Wire Wire Line
	9650 4250 9600 4250
Wire Wire Line
	8650 4400 8600 4400
Wire Wire Line
	8600 4400 8600 4550
Wire Wire Line
	8600 4550 9650 4550
Wire Wire Line
	9650 4550 9650 4950
Wire Wire Line
	9650 4950 9600 4950
Wire Wire Line
	8650 5100 8600 5100
Wire Wire Line
	8600 5100 8600 5250
Wire Wire Line
	8600 5250 9650 5250
Wire Wire Line
	9650 5250 9650 5650
Wire Wire Line
	9650 5650 9600 5650
$Comp
L +3.3V #PWR013
U 1 1 596F2F49
P 8500 3450
F 0 "#PWR013" H 8500 3300 50  0001 C CNN
F 1 "+3.3V" H 8515 3623 50  0000 C CNN
F 2 "" H 8500 3450 50  0001 C CNN
F 3 "" H 8500 3450 50  0001 C CNN
	1    8500 3450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 596F2FA3
P 9750 5900
F 0 "#PWR014" H 9750 5650 50  0001 C CNN
F 1 "GND" H 9755 5727 50  0000 C CNN
F 2 "" H 9750 5900 50  0001 C CNN
F 3 "" H 9750 5900 50  0001 C CNN
	1    9750 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 3550 8500 3550
Wire Wire Line
	8500 3450 8500 5650
Wire Wire Line
	8500 4250 8650 4250
Connection ~ 8500 3550
Wire Wire Line
	8500 4950 8650 4950
Connection ~ 8500 4250
Wire Wire Line
	8500 5650 8650 5650
Connection ~ 8500 4950
Wire Wire Line
	9750 3700 9750 5900
Wire Wire Line
	9750 5800 9600 5800
Wire Wire Line
	9750 5100 9600 5100
Connection ~ 9750 5800
Wire Wire Line
	9750 4400 9600 4400
Connection ~ 9750 5100
Wire Wire Line
	9750 3700 9600 3700
Connection ~ 9750 4400
$Comp
L Battery_Cell BT1
U 1 1 596F363A
P 1100 6850
F 0 "BT1" H 1218 6946 50  0000 L CNN
F 1 "Battery_Cell" H 1218 6855 50  0000 L CNN
F 2 "local:BK-53" V 1100 6910 50  0001 C CNN
F 3 "" V 1100 6910 50  0001 C CNN
	1    1100 6850
	1    0    0    -1  
$EndComp
$Comp
L Battery_Cell BT2
U 1 1 596F3765
P 1100 7250
F 0 "BT2" H 1218 7346 50  0000 L CNN
F 1 "Battery_Cell" H 1218 7255 50  0000 L CNN
F 2 "local:BK-53" V 1100 7310 50  0001 C CNN
F 3 "" V 1100 7310 50  0001 C CNN
	1    1100 7250
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 596F37E7
P 3600 7100
F 0 "C1" H 3715 7146 50  0000 L CNN
F 1 "10u" H 3715 7055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3638 6950 50  0001 C CNN
F 3 "" H 3600 7100 50  0001 C CNN
	1    3600 7100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X05 J3
U 1 1 596F3850
P 2600 7100
F 0 "J3" H 2677 7141 50  0000 L CNN
F 1 "Program" H 2677 7050 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05_Pitch2.54mm" H 2600 7100 50  0001 C CNN
F 3 "" H 2600 7100 50  0001 C CNN
	1    2600 7100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 596F3FDF
P 2400 6900
F 0 "#PWR015" H 2400 6650 50  0001 C CNN
F 1 "GND" V 2405 6772 50  0000 R CNN
F 2 "" H 2400 6900 50  0001 C CNN
F 3 "" H 2400 6900 50  0001 C CNN
	1    2400 6900
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR016
U 1 1 596F405E
P 2400 7200
F 0 "#PWR016" H 2400 7050 50  0001 C CNN
F 1 "+3.3V" V 2415 7328 50  0000 L CNN
F 2 "" H 2400 7200 50  0001 C CNN
F 3 "" H 2400 7200 50  0001 C CNN
	1    2400 7200
	0    -1   -1   0   
$EndComp
Text GLabel 2400 7000 0    60   Input ~ 0
RXD
Text GLabel 2400 7100 0    60   Input ~ 0
TXD
Text GLabel 5100 2350 0    60   Input ~ 0
GPIO0
Wire Wire Line
	5100 2350 5300 2350
Connection ~ 5300 2350
Text GLabel 2400 7300 0    60   Input ~ 0
GPIO0
Wire Wire Line
	1100 7050 1100 6950
$Comp
L +3.3V #PWR017
U 1 1 596F4977
P 1100 6650
F 0 "#PWR017" H 1100 6500 50  0001 C CNN
F 1 "+3.3V" H 1115 6823 50  0000 C CNN
F 2 "" H 1100 6650 50  0001 C CNN
F 3 "" H 1100 6650 50  0001 C CNN
	1    1100 6650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 596F49DD
P 1100 7350
F 0 "#PWR018" H 1100 7100 50  0001 C CNN
F 1 "GND" H 1105 7177 50  0000 C CNN
F 2 "" H 1100 7350 50  0001 C CNN
F 3 "" H 1100 7350 50  0001 C CNN
	1    1100 7350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR019
U 1 1 596F4B1B
P 3600 6950
F 0 "#PWR019" H 3600 6800 50  0001 C CNN
F 1 "+3.3V" H 3615 7123 50  0000 C CNN
F 2 "" H 3600 6950 50  0001 C CNN
F 3 "" H 3600 6950 50  0001 C CNN
	1    3600 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 596F4B58
P 3600 7250
F 0 "#PWR020" H 3600 7000 50  0001 C CNN
F 1 "GND" H 3605 7077 50  0000 C CNN
F 2 "" H 3600 7250 50  0001 C CNN
F 3 "" H 3600 7250 50  0001 C CNN
	1    3600 7250
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 596F4C9A
P 3950 7100
F 0 "C2" H 4065 7146 50  0000 L CNN
F 1 "10u" H 4065 7055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3988 6950 50  0001 C CNN
F 3 "" H 3950 7100 50  0001 C CNN
	1    3950 7100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR021
U 1 1 596F4CA0
P 3950 6950
F 0 "#PWR021" H 3950 6800 50  0001 C CNN
F 1 "+3.3V" H 3965 7123 50  0000 C CNN
F 2 "" H 3950 6950 50  0001 C CNN
F 3 "" H 3950 6950 50  0001 C CNN
	1    3950 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 596F4CA6
P 3950 7250
F 0 "#PWR022" H 3950 7000 50  0001 C CNN
F 1 "GND" H 3955 7077 50  0000 C CNN
F 2 "" H 3950 7250 50  0001 C CNN
F 3 "" H 3950 7250 50  0001 C CNN
	1    3950 7250
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 596F4D54
P 4300 7100
F 0 "C3" H 4415 7146 50  0000 L CNN
F 1 "0.1u" H 4415 7055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4338 6950 50  0001 C CNN
F 3 "" H 4300 7100 50  0001 C CNN
	1    4300 7100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR023
U 1 1 596F4D5A
P 4300 6950
F 0 "#PWR023" H 4300 6800 50  0001 C CNN
F 1 "+3.3V" H 4315 7123 50  0000 C CNN
F 2 "" H 4300 6950 50  0001 C CNN
F 3 "" H 4300 6950 50  0001 C CNN
	1    4300 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 596F4D60
P 4300 7250
F 0 "#PWR024" H 4300 7000 50  0001 C CNN
F 1 "GND" H 4305 7077 50  0000 C CNN
F 2 "" H 4300 7250 50  0001 C CNN
F 3 "" H 4300 7250 50  0001 C CNN
	1    4300 7250
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 596F4D66
P 4650 7100
F 0 "C4" H 4765 7146 50  0000 L CNN
F 1 "0.1u" H 4765 7055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4688 6950 50  0001 C CNN
F 3 "" H 4650 7100 50  0001 C CNN
	1    4650 7100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR025
U 1 1 596F4D6C
P 4650 6950
F 0 "#PWR025" H 4650 6800 50  0001 C CNN
F 1 "+3.3V" H 4665 7123 50  0000 C CNN
F 2 "" H 4650 6950 50  0001 C CNN
F 3 "" H 4650 6950 50  0001 C CNN
	1    4650 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR026
U 1 1 596F4D72
P 4650 7250
F 0 "#PWR026" H 4650 7000 50  0001 C CNN
F 1 "GND" H 4655 7077 50  0000 C CNN
F 2 "" H 4650 7250 50  0001 C CNN
F 3 "" H 4650 7250 50  0001 C CNN
	1    4650 7250
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 J1
U 1 1 596F5B39
P 850 1900
F 0 "J1" H 928 1941 50  0000 L CNN
F 1 "LCD Screen" H 928 1850 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 850 1900 50  0001 C CNN
F 3 "" H 850 1900 50  0001 C CNN
	1    850  1900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1050 1750 2500 1750
NoConn ~ 9800 1400
NoConn ~ 10350 1400
NoConn ~ 8650 5800
$Comp
L D D1
U 1 1 596FFDB1
P 6050 1450
F 0 "D1" V 6004 1529 50  0000 L CNN
F 1 "D" V 6095 1529 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323" H 6050 1450 50  0001 C CNN
F 3 "" H 6050 1450 50  0001 C CNN
	1    6050 1450
	0    1    1    0   
$EndComp
$Comp
L D D3
U 1 1 596FFE7E
P 6500 1450
F 0 "D3" V 6546 1371 50  0000 R CNN
F 1 "D" V 6455 1371 50  0000 R CNN
F 2 "Diodes_SMD:D_SOD-323" H 6500 1450 50  0001 C CNN
F 3 "" H 6500 1450 50  0001 C CNN
	1    6500 1450
	0    -1   -1   0   
$EndComp
$Comp
L D D2
U 1 1 596FFF10
P 6050 1750
F 0 "D2" V 6096 1671 50  0000 R CNN
F 1 "D" V 6005 1671 50  0000 R CNN
F 2 "Diodes_SMD:D_SOD-323" H 6050 1750 50  0001 C CNN
F 3 "" H 6050 1750 50  0001 C CNN
	1    6050 1750
	0    -1   -1   0   
$EndComp
$Comp
L D D8
U 1 1 596FFF78
P 6500 1750
F 0 "D8" V 6454 1829 50  0000 L CNN
F 1 "D" V 6545 1829 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323" H 6500 1750 50  0001 C CNN
F 3 "" H 6500 1750 50  0001 C CNN
	1    6500 1750
	0    1    1    0   
$EndComp
$Comp
L D D9
U 1 1 5970000E
P 7000 2100
F 0 "D9" V 7046 2021 50  0000 R CNN
F 1 "D" V 6955 2021 50  0000 R CNN
F 2 "Diodes_SMD:D_SOD-323" H 7000 2100 50  0001 C CNN
F 3 "" H 7000 2100 50  0001 C CNN
	1    7000 2100
	0    -1   -1   0   
$EndComp
$Comp
L D D10
U 1 1 5970007E
P 7450 2100
F 0 "D10" V 7404 2179 50  0000 L CNN
F 1 "D" V 7495 2179 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-323" H 7450 2100 50  0001 C CNN
F 3 "" H 7450 2100 50  0001 C CNN
	1    7450 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	7200 1250 7200 550 
Wire Wire Line
	7200 550  5800 550 
Wire Wire Line
	5800 550  5800 600 
Wire Wire Line
	6250 600  6250 550 
Connection ~ 6250 550 
Wire Wire Line
	6750 1250 6750 550 
Connection ~ 6750 550 
Wire Wire Line
	7450 2250 7450 2650
Wire Wire Line
	7450 2650 5800 2650
Wire Wire Line
	5800 2650 5800 2600
Wire Wire Line
	6250 2600 6250 2650
Connection ~ 6250 2650
Wire Wire Line
	7000 2250 7000 2650
Connection ~ 7000 2650
Wire Wire Line
	5650 1600 6500 1600
Wire Wire Line
	5800 1900 5500 1900
Wire Wire Line
	5500 1900 5500 1950
Wire Wire Line
	5500 1950 4100 1950
Wire Wire Line
	5800 1300 5800 1850
Wire Wire Line
	5800 1850 4100 1850
Wire Wire Line
	5650 1600 5650 2250
Wire Wire Line
	5650 2250 4100 2250
Connection ~ 6050 1600
Wire Wire Line
	2500 2150 1850 2150
Wire Wire Line
	1850 2150 1850 3450
Wire Wire Line
	1350 2050 2500 2050
Wire Wire Line
	1350 1950 1350 2050
Wire Wire Line
	1450 1950 2500 1950
Wire Wire Line
	1450 1850 1450 1950
Wire Wire Line
	1250 2250 2500 2250
Wire Wire Line
	1850 3450 2250 3450
$EndSCHEMATC
