EESchema Schematic File Version 2
LIBS:FrontEnd_5G-rescue
LIBS:device
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
LIBS:contrib
LIBS:valves
LIBS:ADM7155_Addon-cache
LIBS:LimeMicroBGD_Library
LIBS:power
LIBS:transistors
LIBS:FrontEnd_5G-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 16 20
Title "FrontEnd_5G"
Date "10 Feb 2022"
Rev "1v0"
Comp "Lime Microsystems"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2950 4650 0    59   Input ~ 0
RF_IN
Text HLabel 9100 4650 2    59   Output ~ 0
RF_OUT
Text HLabel 2900 2950 0    59   Input ~ 0
EN
Wire Wire Line
	2900 2950 4050 2950
$Comp
L MNA-7A+ U103
U 1 1 6283332D
P 6150 4600
F 0 "U103" H 6450 5000 60  0000 C CNN
F 1 "MNA-7A+" H 6500 4200 60  0000 C CNN
F 2 "MNA-7A+" H 6150 5450 60  0001 C CNN
F 3 "" H 6250 4900 60  0000 C CNN
F 4 "Mouser" H 6150 6050 60  0001 C CNN "Vendor"
F 5 "139-MNA-7A" H 6150 5950 60  0001 C CNN "Vendor Part Number"
F 6 "Mini-Circuits" H 6150 5850 60  0001 C CNN "Manufacturer"
F 7 "MNA-7A+" H 6150 5750 60  0001 C CNN "Manufacturer Part Number"
F 8 "RF Amplifier MONOLITHIC AMPL / SURF MT/RoHS" H 6150 5650 60  0001 C CNN "Description"
F 9 "MNA-7A+" H 6150 5550 60  0001 C CNN "Component Value"
	1    6150 4600
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01320
U 1 1 62833334
P 6100 5200
F 0 "#PWR01320" H 6100 5250 30  0001 C CNN
F 1 "GND" H 6100 5130 30  0001 C CNN
F 2 "" H 6100 5200 60  0000 C CNN
F 3 "" H 6100 5200 60  0000 C CNN
	1    6100 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5150 6100 5200
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01321
U 1 1 6283333B
P 6200 5200
F 0 "#PWR01321" H 6200 5250 30  0001 C CNN
F 1 "GND" H 6200 5130 30  0001 C CNN
F 2 "" H 6200 5200 60  0000 C CNN
F 3 "" H 6200 5200 60  0000 C CNN
	1    6200 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 5200 6200 5150
Wire Wire Line
	2950 4650 5600 4650
Wire Wire Line
	6700 4650 9100 4650
Wire Wire Line
	6700 4550 7050 4550
Wire Wire Line
	7050 4550 7050 4650
Connection ~ 7050 4650
Wire Wire Line
	5600 4550 5250 4550
Wire Wire Line
	5250 4550 5250 4650
Connection ~ 5250 4650
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01322
U 1 1 6283334C
P 5750 4000
F 0 "#PWR01322" H 5750 4050 30  0001 C CNN
F 1 "GND" H 5750 3930 30  0001 C CNN
F 2 "" H 5750 4000 60  0000 C CNN
F 3 "" H 5750 4000 60  0000 C CNN
	1    5750 4000
	1    0    0    1   
$EndComp
Wire Wire Line
	5750 4050 5750 4000
$Comp
L 1000pF_0402_GCM155R71H102KA37D C457
U 1 1 62833359
P 6700 3300
F 0 "C457" H 6750 3400 50  0000 L CNN
F 1 "1000pF_0402_GCM155R71H102KA37D" H 6000 4000 50  0001 L CNN
F 2 "SMD0402" H 6700 3900 60  0001 C CNN
F 3 "" H 6700 3300 60  0000 C CNN
F 4 "Mouser" H 6650 4500 60  0001 C CNN "Vendor"
F 5 "81-GCM155R71H102KA7D" H 6650 4400 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 6700 4300 60  0001 C CNN "Manufacturer"
F 7 "GCM155R71H102KA37D" H 6700 4200 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 0402 1000pF 50volts X7R 10%" H 6700 4100 60  0001 C CNN "Description"
F 9 "1000pF" H 6825 3200 50  0000 C CNN "Component Value"
	1    6700 3300
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01323
U 1 1 62833360
P 7150 3300
F 0 "#PWR01323" H 7150 3350 30  0001 C CNN
F 1 "GND" H 7150 3230 30  0001 C CNN
F 2 "" H 7150 3300 60  0000 C CNN
F 3 "" H 7150 3300 60  0000 C CNN
	1    7150 3300
	0    -1   1    0   
$EndComp
Wire Wire Line
	6900 3300 7150 3300
Wire Wire Line
	3750 2550 3750 2750
Wire Wire Line
	6100 3300 6500 3300
$Comp
L +5V_RF #PWR01324
U 1 1 6283336A
P 3750 2550
F 0 "#PWR01324" H 3750 2600 20  0001 C CNN
F 1 "+5V_RF" H 3750 2650 30  0000 C CNN
F 2 "" H 3750 2550 60  0000 C CNN
F 3 "" H 3750 2550 60  0000 C CNN
	1    3750 2550
	1    0    0    -1  
$EndComp
Connection ~ 6200 3300
$Comp
L 6.81R_0805_CRCW08056R81FNEA R278
U 1 1 62833377
P 6100 3700
F 0 "R278" H 6090 3770 50  0000 C CNN
F 1 "6.81R_0805_CRCW08056R81FNEA" H 6050 3950 60  0001 C CNN
F 2 "SMD0805" H 6050 4050 60  0001 C CNN
F 3 "" V 6100 3700 60  0000 C CNN
F 4 "Mouser" H 6100 4550 60  0001 C CNN "Vendor"
F 5 "71-CRCW08056R81FNEA" H 6100 4450 60  0001 C CNN "Vendor Part Number"
F 6 "Vishay / Dale" H 6050 4350 60  0001 C CNN "Manufacturer"
F 7 "CRCW08056R81FNEA" H 6050 4250 60  0001 C CNN "Manufacturer Part Number"
F 8 "Thick Film Resistors - SMD 1/8watt 6.81ohms 1% 200ppm" H 6050 4150 60  0001 C CNN "Description"
F 9 "6.81R" H 6100 3690 50  0000 C CNN "Component Value"
	1    6100 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	6100 3450 6100 3300
Wire Wire Line
	6100 3950 6100 4050
$Comp
L TPS22810DBVR U102
U 1 1 62833386
P 4500 2850
F 0 "U102" H 4525 3175 60  0000 C CNN
F 1 "TPS22810DBVR" H 4525 3075 60  0000 C CNN
F 2 "SOT23_TPS22810DBVR" H 4500 3550 60  0001 C CNN
F 3 "" H 4475 2850 60  0000 C CNN
F 4 "Mouser" H 4500 4050 60  0001 C CNN "Vendor"
F 5 "595-TPS22810DBVR" H 4500 3950 60  0001 C CNN "Vendor Part Number"
F 6 "Texas Instruments" H 4550 3850 60  0001 C CNN "Manufacturer"
F 7 "TPS22810DBVR" H 4500 3750 60  0001 C CNN "Manufacturer Part Number"
F 8 "Power Switch ICs - Power Distribution Automotive 18V, 2A" H 4550 3650 60  0001 C CNN "Description"
F 9 "TPS22810DBVR" H 4550 3450 60  0001 C CNN "Component Value"
	1    4500 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2750 4050 2750
Wire Wire Line
	4950 2750 6500 2750
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01325
U 1 1 62833392
P 4000 2850
F 0 "#PWR01325" H 4000 2900 30  0001 C CNN
F 1 "GND" H 4000 2780 30  0001 C CNN
F 2 "" H 4000 2850 60  0000 C CNN
F 3 "" H 4000 2850 60  0000 C CNN
	1    4000 2850
	0    1    -1   0   
$EndComp
Wire Wire Line
	4050 2850 4000 2850
$Comp
L 1nF_0603_06035C102KAT2A C455
U 1 1 6283339F
P 5100 3300
F 0 "C455" H 5150 3400 50  0000 L CNN
F 1 "1nF_0603_06035C102KAT2A" H 4650 3700 50  0001 L CNN
F 2 "SMD0603" H 5100 3600 60  0001 C CNN
F 3 "" H 5100 3300 60  0000 C CNN
F 4 "Mouser" H 5100 4200 60  0001 C CNN "Vendor"
F 5 "581-06035C102K" H 5100 4100 60  0001 C CNN "Vendor Part Number"
F 6 "AVX" H 5100 4000 60  0001 C CNN "Manufacturer"
F 7 "06035C102KAT2A" H 5100 3900 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 50V 1nF X7R 0603 10%TOL" H 5100 3800 60  0001 C CNN "Description"
F 9 "1nF" H 5225 3200 50  0000 C CNN "Component Value"
	1    5100 3300
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01326
U 1 1 628333A6
P 5100 3550
F 0 "#PWR01326" H 5100 3600 30  0001 C CNN
F 1 "GND" H 5100 3480 30  0001 C CNN
F 2 "" H 5100 3550 60  0000 C CNN
F 3 "" H 5100 3550 60  0000 C CNN
	1    5100 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3500 5100 3550
Wire Wire Line
	5100 3100 5100 2950
Wire Wire Line
	5100 2950 4950 2950
$Comp
L 330R_0603_CRGCQ0603J330R R277
U 1 1 628333B5
P 5650 3100
F 0 "R277" H 5640 3170 50  0000 C CNN
F 1 "330R_0603_CRGCQ0603J330R" H 5650 3350 50  0001 C CNN
F 2 "SMD0603" H 5650 3450 60  0001 C CNN
F 3 "" V 5650 3100 60  0000 C CNN
F 4 "Mouser" H 5650 3950 60  0001 C CNN "Vendor"
F 5 "279-CRGCQ0603J330R" H 5650 3850 60  0001 C CNN "Vendor Part Number"
F 6 "TE Connectivity / Holsworthy" H 5650 3750 60  0001 C CNN "Manufacturer"
F 7 "CRGCQ0603J330R" H 5650 3650 60  0001 C CNN "Manufacturer Part Number"
F 8 "Thick Film Resistors - SMD CRGCQ 0603 330R 5% SMD Resistor" H 5650 3550 60  0001 C CNN "Description"
F 9 "330R" H 5650 3090 50  0000 C CNN "Component Value"
	1    5650 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 2850 5650 2750
Connection ~ 5650 2750
Wire Wire Line
	4950 2850 5500 2850
Wire Wire Line
	5500 2850 5500 3450
Wire Wire Line
	5500 3450 5650 3450
Wire Wire Line
	5650 3450 5650 3350
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01327
U 1 1 628333C2
P 7150 2750
F 0 "#PWR01327" H 7150 2800 30  0001 C CNN
F 1 "GND" H 7150 2680 30  0001 C CNN
F 2 "" H 7150 2750 60  0000 C CNN
F 3 "" H 7150 2750 60  0000 C CNN
	1    7150 2750
	0    -1   1    0   
$EndComp
Wire Wire Line
	6900 2750 7150 2750
$Comp
L NF_100nF_0603_C0603C104M5RACTU C456
U 1 1 628333D0
P 6700 2750
F 0 "C456" H 6750 2850 50  0000 L CNN
F 1 "NF_100nF_0603_C0603C104M5RACTU" H 6050 3200 50  0001 L CNN
F 2 "SMD0603" H 6700 3100 60  0001 C CNN
F 3 "" H 6700 2750 60  0000 C CNN
F 4 "Mouser" H 6700 3700 60  0001 C CNN "Vendor"
F 5 "80-C0603C104M5R" H 6650 3600 60  0001 C CNN "Vendor Part Number"
F 6 "KEMET" H 6700 3500 60  0001 C CNN "Manufacturer"
F 7 "C0603C104M5RACTU" H 6700 3400 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 50V 0.1uF 0603 X7R 20%" H 6700 3300 60  0001 C CNN "Description"
F 9 "NF_100nF" H 6825 2650 50  0000 C CNN "Component Value"
	1    6700 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 2750 6200 4050
Connection ~ 6200 2750
$EndSCHEMATC
