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
$Descr A2 23386 16535
encoding utf-8
Sheet 9 20
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
L 10K_0402 R211
U 1 1 6279F8C2
P 3800 4850
F 0 "R211" H 3800 4750 50  0000 C CNN
F 1 "10K_0402" V 3625 4825 50  0001 C CNN
F 2 "SMD0402" V 3500 4825 60  0001 C CNN
F 3 "~" H 3800 4850 60  0000 C CNN
F 4 "Digi-Key" V 3980 4950 60  0001 C CNN "Vendor"
F 5 "RMCF0402FT10K0CT-ND" V 4080 5050 60  0001 C CNN "Vendor Part Number"
F 6 "Stackpole Electronics Inc" V 4180 5150 60  0001 C CNN "Manufacturer"
F 7 "RMCF0402FT10K0" V 4280 5250 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES 10K OHM 1/16W 1% 0402" V 4380 5350 60  0001 C CNN "Description"
F 9 "10K" H 3810 4850 50  0000 C CNN "Component Value"
	1    3800 4850
	0    1    1    0   
$EndComp
Text Label 2900 5500 0    60   ~ 0
SDO
Text Label 2900 5400 0    60   ~ 0
SDIO
Text Label 2900 5300 0    60   ~ 0
SCLK
Text Label 2900 5200 0    60   ~ 0
SAEN
Text Label 3400 6800 0    60   ~ 0
CORE_LDO_EN_#1
$Comp
L LMS8001 U?
U 6 1 6279F8CE
P 5150 5950
AR Path="/55ED8AC7/6279F8CE" Ref="U?"  Part="6" 
AR Path="/6207C085/6279F8CE" Ref="U?"  Part="6" 
AR Path="/6278650D/6279F8CE" Ref="U?"  Part="6" 
AR Path="/6279CAB2/6279F8CE" Ref="U1"  Part="6" 
F 0 "U1" H 5150 6050 60  0000 C CNN
F 1 "LMS8001" H 5150 5750 60  0000 C CNN
F 2 "~" H 5150 6050 60  0000 C CNN
F 3 "~" H 5150 6050 60  0000 C CNN
	6    5150 5950
	-1   0    0    -1  
$EndComp
Text Label 3600 5700 0    60   ~ 0
LMS_GPIO0_#1
Text Label 3600 5800 0    60   ~ 0
LMS_GPIO1_#1
Text Label 3600 5900 0    60   ~ 0
LMS_GPIO2_#1
Text Label 3600 6000 0    60   ~ 0
LMS_GPIO3_#1
Text Label 3600 6200 0    60   ~ 0
LMS_GPIO5_#1
Text Label 3600 6100 0    60   ~ 0
LMS_GPIO4_#1
Text Label 3600 6300 0    60   ~ 0
LMS_GPIO6_#1
Text Label 3600 6400 0    60   ~ 0
LMS_GPIO7_#1
Text Label 3600 6500 0    60   ~ 0
LMS_GPIO8_#1
Text Label 5650 10000 2    60   ~ 0
LMS_GPIO8_#1
Text Label 5650 10350 2    60   ~ 0
LMS_GPIO6_#1
Text Label 5650 10550 2    60   ~ 0
LMS_GPIO4_#1
Text Label 5650 10250 2    60   ~ 0
LMS_GPIO7_#1
Text Label 5650 10450 2    60   ~ 0
LMS_GPIO5_#1
Text Label 5650 10900 2    60   ~ 0
LMS_GPIO2_#1
Text Label 5650 10800 2    60   ~ 0
LMS_GPIO3_#1
Text Label 5650 11000 2    60   ~ 0
LMS_GPIO1_#1
Text Label 5650 11100 2    60   ~ 0
LMS_GPIO0_#1
$Comp
L 330R_0402 R247
U 1 1 6279F8ED
P 4600 10800
F 0 "R247" V 4625 10550 50  0000 C CNN
F 1 "330R_0402" V 4425 10775 50  0001 C CNN
F 2 "SMD0402" V 4300 10775 60  0001 C CNN
F 3 "~" H 4600 10800 60  0000 C CNN
F 4 "Digi-Key" V 4780 10900 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 11000 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 11100 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 11200 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 11300 60  0001 C CNN "Description"
F 9 "330R" V 4610 10800 50  0000 C CNN "Component Value"
	1    4600 10800
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R248
U 1 1 6279F8FA
P 4600 10900
F 0 "R248" V 4625 10650 50  0000 C CNN
F 1 "330R_0402" V 4425 10875 50  0001 C CNN
F 2 "SMD0402" V 4300 10875 60  0001 C CNN
F 3 "~" H 4600 10900 60  0000 C CNN
F 4 "Digi-Key" V 4780 11000 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 11100 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 11200 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 11300 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 11400 60  0001 C CNN "Description"
F 9 "330R" V 4610 10900 50  0000 C CNN "Component Value"
	1    4600 10900
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R249
U 1 1 6279F907
P 4600 11000
F 0 "R249" V 4625 10750 50  0000 C CNN
F 1 "330R_0402" V 4425 10975 50  0001 C CNN
F 2 "SMD0402" V 4300 10975 60  0001 C CNN
F 3 "~" H 4600 11000 60  0000 C CNN
F 4 "Digi-Key" V 4780 11100 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 11200 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 11300 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 11400 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 11500 60  0001 C CNN "Description"
F 9 "330R" V 4610 11000 50  0000 C CNN "Component Value"
	1    4600 11000
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R250
U 1 1 6279F914
P 4600 11100
F 0 "R250" V 4625 10850 50  0000 C CNN
F 1 "330R_0402" V 4425 11075 50  0001 C CNN
F 2 "SMD0402" V 4300 11075 60  0001 C CNN
F 3 "~" H 4600 11100 60  0000 C CNN
F 4 "Digi-Key" V 4780 11200 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 11300 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 11400 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 11500 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 11600 60  0001 C CNN "Description"
F 9 "330R" V 4610 11100 50  0000 C CNN "Component Value"
	1    4600 11100
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R213
U 1 1 6279F921
P 4600 10250
F 0 "R213" V 4625 10000 50  0000 C CNN
F 1 "330R_0402" V 4425 10225 50  0001 C CNN
F 2 "SMD0402" V 4300 10225 60  0001 C CNN
F 3 "~" H 4600 10250 60  0000 C CNN
F 4 "Digi-Key" V 4780 10350 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 10450 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 10550 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 10650 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 10750 60  0001 C CNN "Description"
F 9 "330R" V 4610 10250 50  0000 C CNN "Component Value"
	1    4600 10250
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R214
U 1 1 6279F92E
P 4600 10350
F 0 "R214" V 4625 10100 50  0000 C CNN
F 1 "330R_0402" V 4425 10325 50  0001 C CNN
F 2 "SMD0402" V 4300 10325 60  0001 C CNN
F 3 "~" H 4600 10350 60  0000 C CNN
F 4 "Digi-Key" V 4780 10450 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 10550 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 10650 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 10750 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 10850 60  0001 C CNN "Description"
F 9 "330R" V 4610 10350 50  0000 C CNN "Component Value"
	1    4600 10350
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R244
U 1 1 6279F93B
P 4600 10450
F 0 "R244" V 4625 10200 50  0000 C CNN
F 1 "330R_0402" V 4425 10425 50  0001 C CNN
F 2 "SMD0402" V 4300 10425 60  0001 C CNN
F 3 "~" H 4600 10450 60  0000 C CNN
F 4 "Digi-Key" V 4780 10550 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 10650 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 10750 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 10850 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 10950 60  0001 C CNN "Description"
F 9 "330R" V 4610 10450 50  0000 C CNN "Component Value"
	1    4600 10450
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R245
U 1 1 6279F948
P 4600 10550
F 0 "R245" V 4625 10300 50  0000 C CNN
F 1 "330R_0402" V 4425 10525 50  0001 C CNN
F 2 "SMD0402" V 4300 10525 60  0001 C CNN
F 3 "~" H 4600 10550 60  0000 C CNN
F 4 "Digi-Key" V 4780 10650 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 10750 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 10850 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 10950 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 11050 60  0001 C CNN "Description"
F 9 "330R" V 4610 10550 50  0000 C CNN "Component Value"
	1    4600 10550
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R212
U 1 1 6279F955
P 4600 10000
F 0 "R212" V 4625 9750 50  0000 C CNN
F 1 "330R_0402" V 4425 9975 50  0001 C CNN
F 2 "SMD0402" V 4300 9975 60  0001 C CNN
F 3 "~" H 4600 10000 60  0000 C CNN
F 4 "Digi-Key" V 4780 10100 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 4880 10200 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 4980 10300 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 5080 10400 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 5180 10500 60  0001 C CNN "Description"
F 9 "330R" V 4610 10000 50  0000 C CNN "Component Value"
	1    4600 10000
	0    -1   -1   0   
$EndComp
Text Label 3800 10250 0    60   ~ 0
GPIO7_#1
Text Label 3800 10350 0    60   ~ 0
GPIO6_#1
Text Label 3800 10550 0    60   ~ 0
GPIO4_#1
Text Label 3800 10450 0    60   ~ 0
GPIO5_#1
Text Label 3800 10900 0    60   ~ 0
GPIO2_#1
Text Label 3800 10800 0    60   ~ 0
GPIO3_#1
Text Label 3800 11000 0    60   ~ 0
GPIO1_#1
Text Label 3800 11100 0    60   ~ 0
GPIO0_#1
Text Label 3800 10000 0    60   ~ 0
GPIO8_#1
$Comp
L 10K_0402 R207
U 1 1 6279F96B
P 3250 4850
F 0 "R207" H 3250 4750 50  0000 C CNN
F 1 "10K_0402" V 3075 4825 50  0001 C CNN
F 2 "SMD0402" V 2950 4825 60  0001 C CNN
F 3 "~" H 3250 4850 60  0000 C CNN
F 4 "Digi-Key" V 3430 4950 60  0001 C CNN "Vendor"
F 5 "RMCF0402FT10K0CT-ND" V 3530 5050 60  0001 C CNN "Vendor Part Number"
F 6 "Stackpole Electronics Inc" V 3630 5150 60  0001 C CNN "Manufacturer"
F 7 "RMCF0402FT10K0" V 3730 5250 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES 10K OHM 1/16W 1% 0402" V 3830 5350 60  0001 C CNN "Description"
F 9 "10K" H 3260 4850 50  0000 C CNN "Component Value"
	1    3250 4850
	0    1    1    0   
$EndComp
$Comp
L 10K_0402 R208
U 1 1 6279F978
P 3475 5825
F 0 "R208" H 3500 5750 50  0000 C CNN
F 1 "10K_0402" V 3300 5800 50  0001 C CNN
F 2 "SMD0402" V 3175 5800 60  0001 C CNN
F 3 "~" H 3475 5825 60  0000 C CNN
F 4 "Digi-Key" V 3655 5925 60  0001 C CNN "Vendor"
F 5 "RMCF0402FT10K0CT-ND" V 3755 6025 60  0001 C CNN "Vendor Part Number"
F 6 "Stackpole Electronics Inc" V 3855 6125 60  0001 C CNN "Manufacturer"
F 7 "RMCF0402FT10K0" V 3955 6225 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES 10K OHM 1/16W 1% 0402" V 4055 6325 60  0001 C CNN "Description"
F 9 "10K" H 3485 5825 50  0000 C CNN "Component Value"
	1    3475 5825
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR0680
U 1 1 6279F97F
P 3475 6225
F 0 "#PWR0680" H 3475 6275 30  0001 C CNN
F 1 "GND" H 3475 6155 30  0000 C CNN
F 2 "" H 3475 6225 60  0001 C CNN
F 3 "" H 3475 6225 60  0001 C CNN
	1    3475 6225
	1    0    0    -1  
$EndComp
Text GLabel 2650 5200 0    47   Input ~ 0
LMS8001_#1_SSENn
Text GLabel 2650 5300 0    47   Input ~ 0
SCLK_2
Text GLabel 2650 5400 0    47   BiDi ~ 0
SDIO_2
Text GLabel 2650 5500 0    47   Output ~ 0
SDO_2
Text GLabel 2650 6700 0    47   Output ~ 0
LMS8001_#1_RESETn
Text Label 4700 8500 2    60   ~ 0
CORE_LDO_EN_#1
$Comp
L 10K_0402 R3
U 1 1 6279F991
P 3700 8850
F 0 "R3" H 3690 8920 50  0000 C CNN
F 1 "10K_0402" H 3700 9000 50  0001 C CNN
F 2 "SMD0402" H 3700 9100 60  0001 C CNN
F 3 "" V 3700 8850 60  0000 C CNN
F 4 "Digi-Key" H 3700 9500 60  0001 C CNN "Vendor"
F 5 "-" H 3700 9600 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 3700 9400 60  0001 C CNN "Manufacturer"
F 7 "-" H 3700 9300 60  0001 C CNN "Manufacturer Part Number"
F 8 "-" H 3700 9200 60  0001 C CNN "Description"
F 9 "10K" H 3700 8840 50  0000 C CNN "Component Value"
	1    3700 8850
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR0681
U 1 1 6279F9A5
P 3700 9150
F 0 "#PWR0681" H 3700 9200 30  0001 C CNN
F 1 "GND" H 3700 9080 30  0000 C CNN
F 2 "" H 3700 9150 60  0001 C CNN
F 3 "" H 3700 9150 60  0001 C CNN
	1    3700 9150
	1    0    0    -1  
$EndComp
Text Notes 3900 8850 0    60   ~ 0
This one should be default
Text Notes 3900 8000 0    60   ~ 0
This one should be NF
Text Notes 1775 3750 0    409  ~ 0
LMS8001 #1
$Comp
L 10K_0402 R255
U 1 1 6279F9B4
P 10200 4850
F 0 "R255" H 10200 4750 50  0000 C CNN
F 1 "10K_0402" V 10025 4825 50  0001 C CNN
F 2 "SMD0402" V 9900 4825 60  0001 C CNN
F 3 "~" H 10200 4850 60  0000 C CNN
F 4 "Digi-Key" V 10380 4950 60  0001 C CNN "Vendor"
F 5 "RMCF0402FT10K0CT-ND" V 10480 5050 60  0001 C CNN "Vendor Part Number"
F 6 "Stackpole Electronics Inc" V 10580 5150 60  0001 C CNN "Manufacturer"
F 7 "RMCF0402FT10K0" V 10680 5250 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES 10K OHM 1/16W 1% 0402" V 10780 5350 60  0001 C CNN "Description"
F 9 "10K" H 10210 4850 50  0000 C CNN "Component Value"
	1    10200 4850
	0    1    1    0   
$EndComp
Text Label 9300 5500 0    60   ~ 0
SDO
Text Label 9300 5400 0    60   ~ 0
SDIO
Text Label 9300 5300 0    60   ~ 0
SCLK
Text Label 9300 5200 0    60   ~ 0
SBEN
Text Label 9800 6800 0    60   ~ 0
CORE_LDO_EN_#2
$Comp
L LMS8001 U?
U 6 1 6279F9C0
P 11550 5950
AR Path="/55ED8AC7/6279F9C0" Ref="U?"  Part="6" 
AR Path="/6207C085/6279F9C0" Ref="U?"  Part="6" 
AR Path="/6278650D/6279F9C0" Ref="U?"  Part="6" 
AR Path="/6279CAB2/6279F9C0" Ref="U2"  Part="6" 
F 0 "U2" H 11550 6050 60  0000 C CNN
F 1 "LMS8001" H 11550 5750 60  0000 C CNN
F 2 "~" H 11550 6050 60  0000 C CNN
F 3 "~" H 11550 6050 60  0000 C CNN
	6    11550 5950
	-1   0    0    -1  
$EndComp
Text Label 10000 5700 0    60   ~ 0
LMS_GPIO0_#2
Text Label 10000 5800 0    60   ~ 0
LMS_GPIO1_#2
Text Label 10000 5900 0    60   ~ 0
LMS_GPIO2_#2
Text Label 10000 6000 0    60   ~ 0
LMS_GPIO3_#2
Text Label 10000 6200 0    60   ~ 0
LMS_GPIO5_#2
Text Label 10000 6100 0    60   ~ 0
LMS_GPIO4_#2
Text Label 10000 6300 0    60   ~ 0
LMS_GPIO6_#2
Text Label 10000 6400 0    60   ~ 0
LMS_GPIO7_#2
Text Label 10000 6500 0    60   ~ 0
LMS_GPIO8_#2
$Comp
L 10K_0402 R251
U 1 1 6279F9D6
P 9650 4850
F 0 "R251" H 9650 4750 50  0000 C CNN
F 1 "10K_0402" V 9475 4825 50  0001 C CNN
F 2 "SMD0402" V 9350 4825 60  0001 C CNN
F 3 "~" H 9650 4850 60  0000 C CNN
F 4 "Digi-Key" V 9830 4950 60  0001 C CNN "Vendor"
F 5 "RMCF0402FT10K0CT-ND" V 9930 5050 60  0001 C CNN "Vendor Part Number"
F 6 "Stackpole Electronics Inc" V 10030 5150 60  0001 C CNN "Manufacturer"
F 7 "RMCF0402FT10K0" V 10130 5250 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES 10K OHM 1/16W 1% 0402" V 10230 5350 60  0001 C CNN "Description"
F 9 "10K" H 9660 4850 50  0000 C CNN "Component Value"
	1    9650 4850
	0    1    1    0   
$EndComp
$Comp
L 10K_0402 R252
U 1 1 6279F9E3
P 9875 5825
F 0 "R252" H 9900 5750 50  0000 C CNN
F 1 "10K_0402" V 9700 5800 50  0001 C CNN
F 2 "SMD0402" V 9575 5800 60  0001 C CNN
F 3 "~" H 9875 5825 60  0000 C CNN
F 4 "Digi-Key" V 10055 5925 60  0001 C CNN "Vendor"
F 5 "RMCF0402FT10K0CT-ND" V 10155 6025 60  0001 C CNN "Vendor Part Number"
F 6 "Stackpole Electronics Inc" V 10255 6125 60  0001 C CNN "Manufacturer"
F 7 "RMCF0402FT10K0" V 10355 6225 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES 10K OHM 1/16W 1% 0402" V 10455 6325 60  0001 C CNN "Description"
F 9 "10K" H 9885 5825 50  0000 C CNN "Component Value"
	1    9875 5825
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR0682
U 1 1 6279F9EA
P 9875 6225
F 0 "#PWR0682" H 9875 6275 30  0001 C CNN
F 1 "GND" H 9875 6155 30  0000 C CNN
F 2 "" H 9875 6225 60  0001 C CNN
F 3 "" H 9875 6225 60  0001 C CNN
	1    9875 6225
	1    0    0    -1  
$EndComp
Text GLabel 9050 5200 0    47   Input ~ 0
LMS8001_#2_SSENn
Text GLabel 9050 5300 0    47   Input ~ 0
SCLK_2
Text GLabel 9050 5400 0    47   BiDi ~ 0
SDIO_2
Text GLabel 9050 5500 0    47   Output ~ 0
SDO_2
Text GLabel 9050 6700 0    47   Output ~ 0
LMS8001_#2_RESETn
Text Label 11100 8500 2    60   ~ 0
CORE_LDO_EN_#2
$Comp
L 10K_0402 R4
U 1 1 6279F9FC
P 10100 8850
F 0 "R4" H 10090 8920 50  0000 C CNN
F 1 "10K_0402" H 10100 9000 50  0001 C CNN
F 2 "SMD0402" H 10100 9100 60  0001 C CNN
F 3 "" V 10100 8850 60  0000 C CNN
F 4 "Digi-Key" H 10100 9500 60  0001 C CNN "Vendor"
F 5 "-" H 10100 9600 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 10100 9400 60  0001 C CNN "Manufacturer"
F 7 "-" H 10100 9300 60  0001 C CNN "Manufacturer Part Number"
F 8 "-" H 10100 9200 60  0001 C CNN "Description"
F 9 "10K" H 10100 8840 50  0000 C CNN "Component Value"
	1    10100 8850
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR0683
U 1 1 6279FA10
P 10100 9150
F 0 "#PWR0683" H 10100 9200 30  0001 C CNN
F 1 "GND" H 10100 9080 30  0000 C CNN
F 2 "" H 10100 9150 60  0001 C CNN
F 3 "" H 10100 9150 60  0001 C CNN
	1    10100 9150
	1    0    0    -1  
$EndComp
Text Notes 10300 8800 0    60   ~ 0
This one should be default
Text Notes 10300 7950 0    60   ~ 0
This one should be NF
Text Label 12000 10000 2    60   ~ 0
LMS_GPIO8_#2
Text Label 12000 10350 2    60   ~ 0
LMS_GPIO6_#2
Text Label 12000 10550 2    60   ~ 0
LMS_GPIO4_#2
Text Label 12000 10250 2    60   ~ 0
LMS_GPIO7_#2
Text Label 12000 10450 2    60   ~ 0
LMS_GPIO5_#2
Text Label 12000 10900 2    60   ~ 0
LMS_GPIO2_#2
Text Label 12000 10800 2    60   ~ 0
LMS_GPIO3_#2
Text Label 12000 11000 2    60   ~ 0
LMS_GPIO1_#2
Text Label 12000 11100 2    60   ~ 0
LMS_GPIO0_#2
$Comp
L 330R_0402 R274
U 1 1 6279FA27
P 10950 10800
F 0 "R274" V 10975 10550 50  0000 C CNN
F 1 "330R_0402" V 10775 10775 50  0001 C CNN
F 2 "SMD0402" V 10650 10775 60  0001 C CNN
F 3 "~" H 10950 10800 60  0000 C CNN
F 4 "Digi-Key" V 11130 10900 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 11000 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 11100 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 11200 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 11300 60  0001 C CNN "Description"
F 9 "330R" V 10960 10800 50  0000 C CNN "Component Value"
	1    10950 10800
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R279
U 1 1 6279FA34
P 10950 10900
F 0 "R279" V 10975 10650 50  0000 C CNN
F 1 "330R_0402" V 10775 10875 50  0001 C CNN
F 2 "SMD0402" V 10650 10875 60  0001 C CNN
F 3 "~" H 10950 10900 60  0000 C CNN
F 4 "Digi-Key" V 11130 11000 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 11100 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 11200 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 11300 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 11400 60  0001 C CNN "Description"
F 9 "330R" V 10960 10900 50  0000 C CNN "Component Value"
	1    10950 10900
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R280
U 1 1 6279FA41
P 10950 11000
F 0 "R280" V 10975 10750 50  0000 C CNN
F 1 "330R_0402" V 10775 10975 50  0001 C CNN
F 2 "SMD0402" V 10650 10975 60  0001 C CNN
F 3 "~" H 10950 11000 60  0000 C CNN
F 4 "Digi-Key" V 11130 11100 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 11200 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 11300 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 11400 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 11500 60  0001 C CNN "Description"
F 9 "330R" V 10960 11000 50  0000 C CNN "Component Value"
	1    10950 11000
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R281
U 1 1 6279FA4E
P 10950 11100
F 0 "R281" V 10975 10850 50  0000 C CNN
F 1 "330R_0402" V 10775 11075 50  0001 C CNN
F 2 "SMD0402" V 10650 11075 60  0001 C CNN
F 3 "~" H 10950 11100 60  0000 C CNN
F 4 "Digi-Key" V 11130 11200 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 11300 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 11400 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 11500 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 11600 60  0001 C CNN "Description"
F 9 "330R" V 10960 11100 50  0000 C CNN "Component Value"
	1    10950 11100
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R257
U 1 1 6279FA5B
P 10950 10250
F 0 "R257" V 10975 10000 50  0000 C CNN
F 1 "330R_0402" V 10775 10225 50  0001 C CNN
F 2 "SMD0402" V 10650 10225 60  0001 C CNN
F 3 "~" H 10950 10250 60  0000 C CNN
F 4 "Digi-Key" V 11130 10350 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 10450 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 10550 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 10650 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 10750 60  0001 C CNN "Description"
F 9 "330R" V 10960 10250 50  0000 C CNN "Component Value"
	1    10950 10250
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R258
U 1 1 6279FA68
P 10950 10350
F 0 "R258" V 10975 10100 50  0000 C CNN
F 1 "330R_0402" V 10775 10325 50  0001 C CNN
F 2 "SMD0402" V 10650 10325 60  0001 C CNN
F 3 "~" H 10950 10350 60  0000 C CNN
F 4 "Digi-Key" V 11130 10450 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 10550 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 10650 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 10750 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 10850 60  0001 C CNN "Description"
F 9 "330R" V 10960 10350 50  0000 C CNN "Component Value"
	1    10950 10350
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R272
U 1 1 6279FA75
P 10950 10450
F 0 "R272" V 10975 10200 50  0000 C CNN
F 1 "330R_0402" V 10775 10425 50  0001 C CNN
F 2 "SMD0402" V 10650 10425 60  0001 C CNN
F 3 "~" H 10950 10450 60  0000 C CNN
F 4 "Digi-Key" V 11130 10550 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 10650 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 10750 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 10850 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 10950 60  0001 C CNN "Description"
F 9 "330R" V 10960 10450 50  0000 C CNN "Component Value"
	1    10950 10450
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R273
U 1 1 6279FA82
P 10950 10550
F 0 "R273" V 10975 10300 50  0000 C CNN
F 1 "330R_0402" V 10775 10525 50  0001 C CNN
F 2 "SMD0402" V 10650 10525 60  0001 C CNN
F 3 "~" H 10950 10550 60  0000 C CNN
F 4 "Digi-Key" V 11130 10650 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 10750 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 10850 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 10950 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 11050 60  0001 C CNN "Description"
F 9 "330R" V 10960 10550 50  0000 C CNN "Component Value"
	1    10950 10550
	0    -1   -1   0   
$EndComp
$Comp
L 330R_0402 R256
U 1 1 6279FA8F
P 10950 10000
F 0 "R256" V 10975 9750 50  0000 C CNN
F 1 "330R_0402" V 10775 9975 50  0001 C CNN
F 2 "SMD0402" V 10650 9975 60  0001 C CNN
F 3 "~" H 10950 10000 60  0000 C CNN
F 4 "Digi-Key" V 11130 10100 60  0001 C CNN "Vendor"
F 5 "311-330JRCT-ND" V 11230 10200 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" V 11330 10300 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07330RL" V 11430 10400 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES, 330 OHM, 1/16W, 5%, 0402, SMD," V 11530 10500 60  0001 C CNN "Description"
F 9 "330R" V 10960 10000 50  0000 C CNN "Component Value"
	1    10950 10000
	0    -1   -1   0   
$EndComp
Text Label 10200 10250 0    60   ~ 0
GPIO7_#2
Text Label 10200 10350 0    60   ~ 0
GPIO6_#2
Text Label 10200 10550 0    60   ~ 0
GPIO4_#2
Text Label 10200 10450 0    60   ~ 0
GPIO5_#2
Text Label 10200 10900 0    60   ~ 0
GPIO2_#2
Text Label 10200 10800 0    60   ~ 0
GPIO3_#2
Text Label 10200 11000 0    60   ~ 0
GPIO1_#2
Text Label 10200 11100 0    60   ~ 0
GPIO0_#2
Text Label 10200 10000 0    60   ~ 0
GPIO8_#2
Text Notes 8100 3750 0    409  ~ 0
LMS8001 #2
Wire Wire Line
	4850 10350 5650 10350
Wire Wire Line
	4850 10450 5650 10450
Wire Wire Line
	4850 10550 5650 10550
Wire Wire Line
	4850 10800 5650 10800
Wire Wire Line
	5650 10900 4850 10900
Wire Wire Line
	4850 11000 5650 11000
Wire Wire Line
	4850 11100 5650 11100
Wire Wire Line
	3400 6800 4250 6800
Wire Wire Line
	3600 6500 4250 6500
Wire Wire Line
	4250 5700 3600 5700
Wire Wire Line
	3600 5800 4250 5800
Wire Wire Line
	4250 5900 3600 5900
Wire Wire Line
	4250 6000 3600 6000
Wire Wire Line
	4250 6100 3600 6100
Wire Wire Line
	4250 6200 3600 6200
Wire Wire Line
	4250 6300 3600 6300
Wire Wire Line
	4250 6400 3600 6400
Wire Wire Line
	4850 10250 5650 10250
Wire Wire Line
	3650 10000 4350 10000
Wire Wire Line
	3650 10250 4350 10250
Wire Wire Line
	3650 10350 4350 10350
Wire Wire Line
	3650 10450 4350 10450
Wire Wire Line
	3650 10550 4350 10550
Wire Wire Line
	3650 10800 4350 10800
Wire Wire Line
	3650 10900 4350 10900
Wire Wire Line
	3650 11000 4350 11000
Wire Wire Line
	3650 11100 4350 11100
Wire Wire Line
	5650 10000 4850 10000
Wire Wire Line
	2650 5400 4250 5400
Wire Wire Line
	2650 5500 4250 5500
Wire Wire Line
	2650 5200 4250 5200
Wire Wire Line
	2650 5300 4250 5300
Wire Wire Line
	3800 5100 3800 5200
Connection ~ 3800 5200
Wire Wire Line
	3800 4600 3800 4500
Wire Wire Line
	3250 5100 3250 5500
Wire Wire Line
	3250 4600 3250 4500
Connection ~ 3250 5500
Wire Wire Line
	3475 6075 3475 6225
Wire Wire Line
	3475 5575 3475 5300
Connection ~ 3475 5300
Wire Notes Line
	1275 4100 1275 7350
Wire Notes Line
	1275 7350 5775 7350
Wire Notes Line
	5775 7350 5775 4100
Wire Notes Line
	5775 4100 1275 4100
Wire Notes Line
	2775 9700 5775 9700
Wire Notes Line
	5775 9700 5775 11300
Wire Notes Line
	5775 11300 2775 11300
Wire Notes Line
	2775 11300 2775 9700
Wire Wire Line
	2650 6700 4250 6700
Wire Wire Line
	3700 8500 4700 8500
Wire Wire Line
	3700 8400 3700 8600
Connection ~ 3700 8500
Wire Wire Line
	3700 7900 3700 7850
Wire Wire Line
	3700 9150 3700 9100
Wire Notes Line
	2775 7550 5775 7550
Wire Notes Line
	5775 7550 5775 9450
Wire Notes Line
	5775 9450 2775 9450
Wire Notes Line
	2775 9450 2775 7550
Wire Wire Line
	9800 6800 10650 6800
Wire Wire Line
	10000 6500 10650 6500
Wire Wire Line
	10650 5700 10000 5700
Wire Wire Line
	10000 5800 10650 5800
Wire Wire Line
	10650 5900 10000 5900
Wire Wire Line
	10650 6000 10000 6000
Wire Wire Line
	10650 6100 10000 6100
Wire Wire Line
	10650 6200 10000 6200
Wire Wire Line
	10650 6300 10000 6300
Wire Wire Line
	10650 6400 10000 6400
Wire Wire Line
	9050 5400 10650 5400
Wire Wire Line
	9050 5500 10650 5500
Wire Wire Line
	9050 5200 10650 5200
Wire Wire Line
	9050 5300 10650 5300
Wire Wire Line
	10200 5100 10200 5200
Connection ~ 10200 5200
Wire Wire Line
	10200 4600 10200 4500
Wire Wire Line
	9650 5100 9650 5500
Wire Wire Line
	9650 4600 9650 4500
Connection ~ 9650 5500
Wire Wire Line
	9875 6075 9875 6225
Wire Wire Line
	9875 5575 9875 5300
Connection ~ 9875 5300
Wire Notes Line
	7650 4100 7650 7350
Wire Notes Line
	7650 7350 12150 7350
Wire Notes Line
	12150 7350 12150 4100
Wire Notes Line
	12150 4100 7650 4100
Wire Wire Line
	9050 6700 10650 6700
Wire Wire Line
	10100 8500 11100 8500
Wire Wire Line
	10100 8400 10100 8600
Connection ~ 10100 8500
Wire Wire Line
	10100 7900 10100 7850
Wire Wire Line
	10100 9150 10100 9100
Wire Notes Line
	9150 7550 12150 7550
Wire Notes Line
	12150 7550 12150 9450
Wire Notes Line
	12150 9450 9150 9450
Wire Notes Line
	9150 9450 9150 7550
Wire Wire Line
	11200 10350 12000 10350
Wire Wire Line
	11200 10450 12000 10450
Wire Wire Line
	11200 10550 12000 10550
Wire Wire Line
	11200 10800 12000 10800
Wire Wire Line
	12000 10900 11200 10900
Wire Wire Line
	11200 11000 12000 11000
Wire Wire Line
	11200 11100 12000 11100
Wire Wire Line
	11200 10250 12000 10250
Wire Wire Line
	10050 10000 10700 10000
Wire Wire Line
	10050 10250 10700 10250
Wire Wire Line
	10050 10350 10700 10350
Wire Wire Line
	10050 10450 10700 10450
Wire Wire Line
	10050 10550 10700 10550
Wire Wire Line
	10050 10800 10700 10800
Wire Wire Line
	10050 10900 10700 10900
Wire Wire Line
	10050 11000 10700 11000
Wire Wire Line
	10050 11100 10700 11100
Wire Wire Line
	12000 10000 11200 10000
Wire Notes Line
	9150 9700 12150 9700
Wire Notes Line
	12150 9700 12150 11300
Wire Notes Line
	12150 11300 9150 11300
Wire Notes Line
	9150 11300 9150 9700
Wire Notes Line
	850  2725 12475 2725
Wire Notes Line
	12475 2725 12475 11650
Wire Notes Line
	12475 11650 850  11650
Wire Notes Line
	850  11650 850  2725
Wire Notes Line
	6700 2950 6700 11550
$Comp
L NF_10K_0402 R209
U 1 1 62841918
P 3700 8150
F 0 "R209" H 3690 8220 50  0000 C CNN
F 1 "NF_10K_0402" H 3700 8300 50  0001 C CNN
F 2 "SMD0402" H 3700 8400 60  0001 C CNN
F 3 "" V 3700 8150 60  0000 C CNN
F 4 "Digi-Key" H 3700 8800 60  0001 C CNN "Vendor"
F 5 "-" H 3700 8900 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 3700 8700 60  0001 C CNN "Manufacturer"
F 7 "-" H 3700 8600 60  0001 C CNN "Manufacturer Part Number"
F 8 "-" H 3700 8500 60  0001 C CNN "Description"
F 9 "NF_10K" H 3700 8140 50  0000 C CNN "Component Value"
	1    3700 8150
	0    1    1    0   
$EndComp
$Comp
L NF_10K_0402 R253
U 1 1 62842B32
P 10100 8150
F 0 "R253" H 10090 8220 50  0000 C CNN
F 1 "NF_10K_0402" H 10100 8300 50  0001 C CNN
F 2 "SMD0402" H 10100 8400 60  0001 C CNN
F 3 "" V 10100 8150 60  0000 C CNN
F 4 "Digi-Key" H 10100 8800 60  0001 C CNN "Vendor"
F 5 "-" H 10100 8900 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 10100 8700 60  0001 C CNN "Manufacturer"
F 7 "-" H 10100 8600 60  0001 C CNN "Manufacturer Part Number"
F 8 "-" H 10100 8500 60  0001 C CNN "Description"
F 9 "NF_10K" H 10100 8140 50  0000 C CNN "Component Value"
	1    10100 8150
	0    1    1    0   
$EndComp
NoConn ~ 10050 10000
NoConn ~ 3650 10000
NoConn ~ 10050 10250
NoConn ~ 10050 10350
NoConn ~ 10050 10450
NoConn ~ 10050 10550
NoConn ~ 10050 10800
NoConn ~ 10050 10900
NoConn ~ 10050 11000
NoConn ~ 10050 11100
NoConn ~ 3650 11100
NoConn ~ 3650 11000
NoConn ~ 3650 10900
NoConn ~ 3650 10800
NoConn ~ 3650 10550
NoConn ~ 3650 10450
NoConn ~ 3650 10350
NoConn ~ 3650 10250
$Comp
L +3.3V_#1 #PWR0684
U 1 1 6347271A
P 3250 4500
F 0 "#PWR0684" H 3250 4640 20  0001 C CNN
F 1 "+3.3V_#1" H 3250 4600 20  0000 C CNN
F 2 "" H 3250 4500 60  0000 C CNN
F 3 "" H 3250 4500 60  0000 C CNN
	1    3250 4500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V_#1 #PWR0685
U 1 1 634728FC
P 3800 4500
F 0 "#PWR0685" H 3800 4640 20  0001 C CNN
F 1 "+3.3V_#1" H 3800 4600 20  0000 C CNN
F 2 "" H 3800 4500 60  0000 C CNN
F 3 "" H 3800 4500 60  0000 C CNN
	1    3800 4500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V_#1 #PWR0686
U 1 1 634731AE
P 3700 7850
F 0 "#PWR0686" H 3700 7990 20  0001 C CNN
F 1 "+3.3V_#1" H 3700 7950 20  0000 C CNN
F 2 "" H 3700 7850 60  0000 C CNN
F 3 "" H 3700 7850 60  0000 C CNN
	1    3700 7850
	1    0    0    -1  
$EndComp
$Comp
L +3.3V_#2 #PWR0687
U 1 1 63476588
P 9650 4500
F 0 "#PWR0687" H 9650 4640 20  0001 C CNN
F 1 "+3.3V_#2" H 9650 4600 20  0000 C CNN
F 2 "" H 9650 4500 60  0000 C CNN
F 3 "" H 9650 4500 60  0000 C CNN
	1    9650 4500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V_#2 #PWR0688
U 1 1 634768A6
P 10200 4500
F 0 "#PWR0688" H 10200 4640 20  0001 C CNN
F 1 "+3.3V_#2" H 10200 4600 20  0000 C CNN
F 2 "" H 10200 4500 60  0000 C CNN
F 3 "" H 10200 4500 60  0000 C CNN
	1    10200 4500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V_#2 #PWR0689
U 1 1 634771D1
P 10100 7850
F 0 "#PWR0689" H 10100 7990 20  0001 C CNN
F 1 "+3.3V_#2" H 10100 7950 20  0000 C CNN
F 2 "" H 10100 7850 60  0000 C CNN
F 3 "" H 10100 7850 60  0000 C CNN
	1    10100 7850
	1    0    0    -1  
$EndComp
$EndSCHEMATC