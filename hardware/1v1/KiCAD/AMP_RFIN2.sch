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
Sheet 7 20
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 1550 3750 0    59   Input ~ 0
RF_IN
Text HLabel 9150 3750 2    59   Output ~ 0
RF_OUT
Text HLabel 1550 5600 0    59   Input ~ 0
BYP
Wire Wire Line
	6350 1700 6350 3050
$Comp
L +5V_RF #PWR0538
U 1 1 626B56ED
P 6350 1700
F 0 "#PWR0538" H 6350 1750 20  0001 C CNN
F 1 "+5V_RF" H 6350 1800 30  0000 C CNN
F 2 "" H 6350 1700 60  0000 C CNN
F 3 "" H 6350 1700 60  0000 C CNN
	1    6350 1700
	1    0    0    -1  
$EndComp
$Comp
L TQL9063 U106
U 1 1 626B56F9
P 5500 3850
F 0 "U106" H 5250 3400 60  0000 C CNN
F 1 "TQL9063" H 5750 3400 60  0000 C CNN
F 2 "TQL9063" H 5500 5100 60  0001 C CNN
F 3 "" H 5350 3850 60  0000 C CNN
F 4 "Mouser" H 5500 5200 60  0001 C CNN "Vendor"
F 5 "772-TQL9063" H 5500 5300 60  0001 C CNN "Vendor Part Number"
F 6 "Qorvo" H 5500 5400 60  0001 C CNN "Manufacturer"
F 7 "TQL9063" H 5500 5500 60  0001 C CNN "Manufacturer Part Number"
F 8 "RF Amplifier 2.3-5.0 GHz High Gain 0.5W Driver Amplif" H 5500 5600 60  0001 C CNN "Description"
F 9 "TQL9063" H 5500 5000 60  0001 C CNN "Component Value"
	1    5500 3850
	1    0    0    -1  
$EndComp
Text Notes 2600 4000 0    59   ~ 0
C1=100pF
Wire Wire Line
	7850 3750 6050 3750
Text Notes 7825 4000 0    59   ~ 0
C2=27pF
$Comp
L 27pF_0402_GCM1555C1H270FA16D C623
U 1 1 626B5709
P 8050 3750
F 0 "C623" V 7900 3850 50  0000 L CNN
F 1 "27pF_0402_GCM1555C1H270FA16D" H 7700 4150 50  0001 L CNN
F 2 "SMD0402" H 8050 4050 39  0001 C CNN
F 3 "" H 8100 3850 60  0000 C CNN
F 4 "Mouser" H 8050 4250 60  0001 C CNN "Vendor"
F 5 "81-GCM1555C1H270FA6D" H 8000 4350 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 8050 4450 60  0001 C CNN "Manufacturer"
F 7 "GCM1555C1H270FA16D" H 8100 4550 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 0402 27pF 50volts C0G 1%" H 8150 4650 60  0001 C CNN "Description"
F 9 "27pF" V 7900 3600 50  0000 C CNN "Component Value"
	1    8050 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 3750 4950 3750
$Comp
L 0R_0402 R577
U 1 1 626B5717
P 4700 5000
F 0 "R577" V 4650 5100 50  0000 C CNN
F 1 "0R_0402" H 4700 5150 50  0001 C CNN
F 2 "SMD0402" H 4700 5250 60  0001 C CNN
F 3 "" V 4700 5000 60  0000 C CNN
F 4 "Digi-Key" H 4700 5650 60  0001 C CNN "Vendor"
F 5 "-" H 4700 5750 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 4700 5550 60  0001 C CNN "Manufacturer"
F 7 "-" H 4700 5450 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES_0.0_OHM_0402_SMD" H 4700 5350 60  0001 C CNN "Description"
F 9 "0R" H 4700 4990 50  0000 C CNN "Component Value"
	1    4700 5000
	0    1    1    0   
$EndComp
$Comp
L GND #PWR0539
U 1 1 626B571E
P 3800 4550
F 0 "#PWR0539" H 3800 4550 30  0001 C CNN
F 1 "GND" H 3800 4480 30  0001 C CNN
F 2 "" H 3800 4550 60  0000 C CNN
F 3 "" H 3800 4550 60  0000 C CNN
	1    3800 4550
	0    1    1    0   
$EndComp
$Comp
L GND #PWR0540
U 1 1 626B5724
P 4775 3550
F 0 "#PWR0540" H 4775 3550 30  0001 C CNN
F 1 "GND" H 4775 3480 30  0001 C CNN
F 2 "" H 4775 3550 60  0000 C CNN
F 3 "" H 4775 3550 60  0000 C CNN
	1    4775 3550
	0    1    1    0   
$EndComp
$Comp
L GND #PWR0541
U 1 1 626B572A
P 5500 4600
F 0 "#PWR0541" H 5500 4600 30  0001 C CNN
F 1 "GND" H 5500 4530 30  0001 C CNN
F 2 "" H 5500 4600 60  0000 C CNN
F 3 "" H 5500 4600 60  0000 C CNN
	1    5500 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 4400 5500 4600
Wire Wire Line
	4950 3550 4775 3550
Wire Wire Line
	4950 3950 4700 3950
Wire Wire Line
	4700 3950 4700 4750
Wire Wire Line
	4400 4550 4700 4550
Connection ~ 4700 4550
Wire Wire Line
	4000 4550 3800 4550
Wire Wire Line
	4700 5600 4700 5250
Text Notes 3925 4800 0    59   ~ 0
C7=100pF
Text Notes 4750 5100 0    59   ~ 0
R2=0 R
Wire Wire Line
	6050 3950 6350 3950
Wire Wire Line
	6350 3950 6350 4750
Wire Wire Line
	6650 4550 6350 4550
Connection ~ 6350 4550
$Comp
L 0R_0402 R579
U 1 1 626B5746
P 6350 5000
F 0 "R579" V 6300 5100 50  0000 C CNN
F 1 "0R_0402" H 6350 5150 50  0001 C CNN
F 2 "SMD0402" H 6350 5250 60  0001 C CNN
F 3 "" V 6350 5000 60  0000 C CNN
F 4 "Digi-Key" H 6350 5650 60  0001 C CNN "Vendor"
F 5 "-" H 6350 5750 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 6350 5550 60  0001 C CNN "Manufacturer"
F 7 "-" H 6350 5450 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES_0.0_OHM_0402_SMD" H 6350 5350 60  0001 C CNN "Description"
F 9 "0R" H 6350 4990 50  0000 C CNN "Component Value"
	1    6350 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	6350 6750 6350 5250
$Comp
L GND #PWR0542
U 1 1 626B574E
P 7250 4550
F 0 "#PWR0542" H 7250 4550 30  0001 C CNN
F 1 "GND" H 7250 4480 30  0001 C CNN
F 2 "" H 7250 4550 60  0000 C CNN
F 3 "" H 7250 4550 60  0000 C CNN
	1    7250 4550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7250 4550 7050 4550
NoConn ~ 6050 4150
NoConn ~ 4950 4150
Text Notes 6625 4750 0    59   ~ 0
C8=100pF
Text Notes 6400 5100 0    59   ~ 0
R3=0 R
$Comp
L 68nH_0603_0603CS-68NXJLW L48
U 1 1 626B575F
P 6350 3250
F 0 "L48" H 6250 3250 50  0000 C CNN
F 1 "68nH_0603_0603CS-68NXJLW" H 6350 3500 50  0001 C CNN
F 2 "SMD0603" H 6350 3600 60  0001 C CNN
F 3 "" H 6350 3250 60  0000 C CNN
F 4 "Mouser" H 6350 4100 60  0001 C CNN "Vendor"
F 5 "994-0603CS-68NXJLW" H 6350 4000 60  0001 C CNN "Vendor Part Number"
F 6 "Coilcraft" H 6350 3900 60  0001 C CNN "Manufacturer"
F 7 "0603CS-68NXJLW" H 6350 3800 60  0001 C CNN "Manufacturer Part Number"
F 8 "Fixed Inductors 0603HP Hi Perfrmnce 100 nH 5 % 0.47 A" H 6350 3700 60  0001 C CNN "Description"
F 9 "68nH" H 6200 3150 50  0000 C CNN "Component Value"
	1    6350 3250
	1    0    0    -1  
$EndComp
Text Notes 6400 3300 0    59   ~ 0
L1=68nH
$Comp
L 1000pF_0402_GCM155R71H102KA37D C621
U 1 1 626B576D
P 6800 2950
F 0 "C621" V 6850 3000 50  0000 L CNN
F 1 "1000pF_0402_GCM155R71H102KA37D" H 6100 3650 50  0001 L CNN
F 2 "SMD0402" H 6800 3550 60  0001 C CNN
F 3 "" H 6800 2950 60  0000 C CNN
F 4 "Mouser" H 6750 4150 60  0001 C CNN "Vendor"
F 5 "81-GCM155R71H102KA7D" H 6750 4050 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 6800 3950 60  0001 C CNN "Manufacturer"
F 7 "GCM155R71H102KA37D" H 6800 3850 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 0402 1000pF 50volts X7R 10%" H 6800 3750 60  0001 C CNN "Description"
F 9 "1000pF" V 7000 2950 50  0000 C CNN "Component Value"
	1    6800 2950
	0    1    1    0   
$EndComp
$Comp
L 1uF_0402_CC0402ZRY5V5BB105 C620
U 1 1 626B577A
P 6800 2500
F 0 "C620" V 6900 2550 50  0000 L CNN
F 1 "1uF_0402_CC0402ZRY5V5BB105" H 6200 3050 50  0001 L CNN
F 2 "SMD0402" H 6800 2950 60  0001 C CNN
F 3 "" H 6800 2500 60  0000 C CNN
F 4 "Mouser" H 6800 3550 60  0001 C CNN "Vendor"
F 5 "603-CC402ZRY5V5BB105" H 6800 3450 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" H 6800 3350 60  0001 C CNN "Manufacturer"
F 7 "CC0402ZRY5V5BB105" H 6800 3250 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 1.0uF 6.3V Y5V -20/+80%" H 6800 3150 60  0001 C CNN "Description"
F 9 "1uF" V 6900 2350 50  0000 C CNN "Component Value"
	1    6800 2500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR0543
U 1 1 626B578E
P 7200 2950
F 0 "#PWR0543" H 7200 2950 30  0001 C CNN
F 1 "GND" H 7200 2880 30  0001 C CNN
F 2 "" H 7200 2950 60  0000 C CNN
F 3 "" H 7200 2950 60  0000 C CNN
	1    7200 2950
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR0544
U 1 1 626B5794
P 7200 2500
F 0 "#PWR0544" H 7200 2500 30  0001 C CNN
F 1 "GND" H 7200 2430 30  0001 C CNN
F 2 "" H 7200 2500 60  0000 C CNN
F 3 "" H 7200 2500 60  0000 C CNN
	1    7200 2500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6050 3550 6350 3550
Wire Wire Line
	6350 3550 6350 3450
Wire Wire Line
	6350 2950 6600 2950
Wire Wire Line
	6350 2500 6600 2500
Connection ~ 6350 2950
Wire Wire Line
	7000 2500 7200 2500
Wire Wire Line
	7000 2950 7200 2950
Text Notes 6525 2825 0    59   ~ 0
C6=1000pF
Text Notes 6625 2350 0    59   ~ 0
C5=1uF
Connection ~ 6350 2500
$Comp
L 100pF_0402_ECJ-0EC1H101J C618
U 1 1 626B57AD
P 2850 3750
F 0 "C618" V 2950 3850 50  0000 L CNN
F 1 "100pF_0402_ECJ-0EC1H101J" H 2400 4150 50  0001 L CNN
F 2 "SMD0402" H 2850 4050 39  0001 C CNN
F 3 "" H 2900 3850 60  0000 C CNN
F 4 "Mouser" H 2850 4250 60  0001 C CNN "Vendor"
F 5 "667-ECJ-0EC1H101J" H 2800 4350 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 2850 4450 60  0001 C CNN "Manufacturer"
F 7 "ECJ-0EC1H101J" H 2900 4550 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100PF 50V 5% 0402" H 2950 4650 60  0001 C CNN "Description"
F 9 "100pF" V 2650 3750 50  0000 C CNN "Component Value"
	1    2850 3750
	0    1    1    0   
$EndComp
$Comp
L 100pF_0402_ECJ-0EC1H101J C622
U 1 1 626B57BA
P 6850 4550
F 0 "C622" V 6950 4650 50  0000 L CNN
F 1 "100pF_0402_ECJ-0EC1H101J" H 6400 4950 50  0001 L CNN
F 2 "SMD0402" H 6850 4850 39  0001 C CNN
F 3 "" H 6900 4650 60  0000 C CNN
F 4 "Mouser" H 6850 5050 60  0001 C CNN "Vendor"
F 5 "667-ECJ-0EC1H101J" H 6800 5150 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 6850 5250 60  0001 C CNN "Manufacturer"
F 7 "ECJ-0EC1H101J" H 6900 5350 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100PF 50V 5% 0402" H 6950 5450 60  0001 C CNN "Description"
F 9 "100pF" V 6650 4550 50  0000 C CNN "Component Value"
	1    6850 4550
	0    1    1    0   
$EndComp
$Comp
L 100pF_0402_ECJ-0EC1H101J C619
U 1 1 626B57C7
P 4200 4550
F 0 "C619" V 4300 4650 50  0000 L CNN
F 1 "100pF_0402_ECJ-0EC1H101J" H 3750 4950 50  0001 L CNN
F 2 "SMD0402" H 4200 4850 39  0001 C CNN
F 3 "" H 4250 4650 60  0000 C CNN
F 4 "Mouser" H 4200 5050 60  0001 C CNN "Vendor"
F 5 "667-ECJ-0EC1H101J" H 4150 5150 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 4200 5250 60  0001 C CNN "Manufacturer"
F 7 "ECJ-0EC1H101J" H 4250 5350 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100PF 50V 5% 0402" H 4300 5450 60  0001 C CNN "Description"
F 9 "100pF" V 4000 4550 50  0000 C CNN "Component Value"
	1    4200 4550
	0    1    1    0   
$EndComp
Text HLabel 1550 6750 0    59   Input ~ 0
SD
$Comp
L 10K_0402 R299
U 1 1 626B57D6
P 3750 5900
F 0 "R299" H 3740 5970 50  0000 C CNN
F 1 "10K_0402" H 3750 6050 50  0001 C CNN
F 2 "SMD0402" H 3750 6150 60  0001 C CNN
F 3 "" V 3750 5900 60  0000 C CNN
F 4 "Digi-Key" H 3750 6550 60  0001 C CNN "Vendor"
F 5 "-" H 3750 6650 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 3750 6450 60  0001 C CNN "Manufacturer"
F 7 "-" H 3750 6350 60  0001 C CNN "Manufacturer Part Number"
F 8 "-" H 3750 6250 60  0001 C CNN "Description"
F 9 "10K" H 3750 5890 50  0000 C CNN "Component Value"
	1    3750 5900
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR0545
U 1 1 626B57DD
P 3750 6200
F 0 "#PWR0545" H 3750 6200 30  0001 C CNN
F 1 "GND" H 3750 6130 30  0001 C CNN
F 2 "" H 3750 6200 60  0000 C CNN
F 3 "" H 3750 6200 60  0000 C CNN
	1    3750 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 6150 3750 6200
Wire Wire Line
	3750 5650 3750 5600
Connection ~ 3750 5600
$Comp
L 10K_0402 R576
U 1 1 626B57EC
P 3750 7050
F 0 "R576" H 3740 7120 50  0000 C CNN
F 1 "10K_0402" H 3750 7200 50  0001 C CNN
F 2 "SMD0402" H 3750 7300 60  0001 C CNN
F 3 "" V 3750 7050 60  0000 C CNN
F 4 "Digi-Key" H 3750 7700 60  0001 C CNN "Vendor"
F 5 "-" H 3750 7800 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 3750 7600 60  0001 C CNN "Manufacturer"
F 7 "-" H 3750 7500 60  0001 C CNN "Manufacturer Part Number"
F 8 "-" H 3750 7400 60  0001 C CNN "Description"
F 9 "10K" H 3750 7040 50  0000 C CNN "Component Value"
	1    3750 7050
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR0546
U 1 1 626B57F3
P 3750 7350
F 0 "#PWR0546" H 3750 7350 30  0001 C CNN
F 1 "GND" H 3750 7280 30  0001 C CNN
F 2 "" H 3750 7350 60  0000 C CNN
F 3 "" H 3750 7350 60  0000 C CNN
	1    3750 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 7300 3750 7350
Wire Wire Line
	3750 6800 3750 6750
Connection ~ 3750 6750
Wire Wire Line
	3550 6750 6350 6750
Wire Wire Line
	3550 5600 4700 5600
$Comp
L 10K_0402 R297
U 1 1 626B5806
P 3300 5600
F 0 "R297" H 3290 5670 50  0000 C CNN
F 1 "10K_0402" H 3300 5750 50  0001 C CNN
F 2 "SMD0402" H 3300 5850 60  0001 C CNN
F 3 "" V 3300 5600 60  0000 C CNN
F 4 "Digi-Key" H 3300 6250 60  0001 C CNN "Vendor"
F 5 "-" H 3300 6350 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 3300 6150 60  0001 C CNN "Manufacturer"
F 7 "-" H 3300 6050 60  0001 C CNN "Manufacturer Part Number"
F 8 "-" H 3300 5950 60  0001 C CNN "Description"
F 9 "10K" H 3300 5590 50  0000 C CNN "Component Value"
	1    3300 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 5600 1550 5600
$Comp
L 10K_0402 R298
U 1 1 626B5814
P 3300 6750
F 0 "R298" H 3290 6820 50  0000 C CNN
F 1 "10K_0402" H 3300 6900 50  0001 C CNN
F 2 "SMD0402" H 3300 7000 60  0001 C CNN
F 3 "" V 3300 6750 60  0000 C CNN
F 4 "Digi-Key" H 3300 7400 60  0001 C CNN "Vendor"
F 5 "-" H 3300 7500 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 3300 7300 60  0001 C CNN "Manufacturer"
F 7 "-" H 3300 7200 60  0001 C CNN "Manufacturer Part Number"
F 8 "-" H 3300 7100 60  0001 C CNN "Description"
F 9 "10K" H 3300 6740 50  0000 C CNN "Component Value"
	1    3300 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 6750 1550 6750
Wire Notes Line
	3050 5350 3950 5350
Wire Notes Line
	3950 5350 3950 6300
Wire Notes Line
	3950 6300 3050 6300
Wire Notes Line
	3050 6300 3050 5350
Wire Notes Line
	3050 6500 3950 6500
Wire Notes Line
	3950 6500 3950 7450
Wire Notes Line
	3050 7450 3050 6500
Wire Notes Line
	3950 7450 3050 7450
Text Notes 2700 6450 0    59   ~ 0
Voltage devider and pull-down resistor
Text Notes 2650 5300 0    59   ~ 0
Voltage devider and pull-down resistor
Text Notes 1300 5050 0    59   ~ 0
BYP=0,  SD=0  => LNA ON,  Bypass OFF\nBYP=0,  SD=1  => LNA OFF,  Bypass OFF\nBYP=1,  SD=X  => LNA OFF,  Bypass ON
Wire Wire Line
	1550 3750 2650 3750
Wire Wire Line
	8250 3750 9150 3750
$EndSCHEMATC
