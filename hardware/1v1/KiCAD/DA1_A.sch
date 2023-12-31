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
$Descr A3 16535 11693
encoding utf-8
Sheet 17 20
Title "FrontEnd_5G"
Date "10 Feb 2022"
Rev "1v0"
Comp "Lime Microsystems"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 100uF_63V_Electrolytic_Panasonic C203
U 1 1 6220402C
P 7575 2625
F 0 "C203" H 7675 2725 50  0000 L CNN
F 1 "100uF_63V_Electrolytic_Panasonic" H 7025 2925 50  0001 L CNN
F 2 "LimeMicroBGD_Library:EEE-FK1J101P" H 7575 3025 50  0001 C CNN
F 3 "" H 7600 2725 50  0001 C CNN
F 4 "Mouser" H 7575 3525 60  0001 C CNN "Vendor"
F 5 "667-EEE-FK1J101P" H 7575 3425 60  0001 C CNN "Vendor Part Number"
F 6 "Panasonic" H 7575 3325 60  0001 C CNN "Manufacturer"
F 7 "EEE-FK1J101P" H 7575 3225 60  0001 C CNN "Manufacturer Part Number"
F 8 "Aluminium Electrolytic Capacitors - SMD 100UF 63V FK SMD" H 7675 3125 60  0001 C CNN "Description"
F 9 "100uF" H 7725 2525 60  0000 C CNN "Component Value"
	1    7575 2625
	0    -1   -1   0   
$EndComp
Text Notes 6900 1150 2    120  ~ 24
Vcc=5V
Text Notes 10525 8700 2    120  ~ 24
TDD_ctrl=3-5V
NoConn ~ 4775 7475
NoConn ~ 4775 7075
NoConn ~ 6675 6675
NoConn ~ 5825 6225
NoConn ~ 5425 6225
NoConn ~ 6025 8125
NoConn ~ 6225 8125
NoConn ~ 5225 8125
$Comp
L 1.2nH_0402_0402CS-1N2XJLU L13
U 1 1 62204043
P 3450 7275
F 0 "L13" V 3365 7305 50  0000 C CNN
F 1 "1.2nH_0402_0402CS-1N2XJLU" H 3450 7775 50  0001 C CNN
F 2 "SMD0402" H 3500 7675 60  0001 C CNN
F 3 "" V 3365 7305 60  0000 C CNN
F 4 "Mouser" H 3500 8275 60  0001 C CNN "Vendor"
F 5 "994-0402CS-1N2XJLU" H 3500 8175 60  0001 C CNN "Vendor Part Number"
F 6 "Coilcraft" H 3400 7975 60  0001 C CNN "Manufacturer"
F 7 "0402CS-1N2XJLU" H 3450 8075 60  0001 C CNN "Manufacturer Part Number"
F 8 "Fixed Inductors 1005 1.2nH Unshld 5% 740mA 90mOhms AECQ2" H 3400 7875 60  0001 C CNN "Description"
F 9 "1.2nH" V 3515 7255 50  0000 C CNN "Component Value"
	1    3450 7275
	0    1    1    0   
$EndComp
$Comp
L 5.6pF_0402_04023J5R6BBWTR C191
U 1 1 62204050
P 4150 7275
F 0 "C191" H 4200 7375 50  0000 L CNN
F 1 "5.6pF_0402_04023J5R6BBWTR" H 3550 7725 50  0001 L CNN
F 2 "SMD0402" H 4150 7625 47  0001 C CNN
F 3 "" H 4200 7375 60  0000 C CNN
F 4 "Mouser" H 4150 8225 60  0001 C CNN "Vendor"
F 5 "581-04023J5R6BBW" H 4150 8125 60  0001 C CNN "Vendor Part Number"
F 6 "AVX" H 4150 8025 60  0001 C CNN "Manufacturer"
F 7 "04023J5R6BBWTR" H 4150 7925 60  0001 C CNN "Manufacturer Part Number"
F 8 "Silicon RF Capacitors / Thin Film 25V 5.6pF .1pFTol ThinFilm 0402" H 4050 7825 60  0001 C CNN "Description"
F 9 "5.6pF" H 4275 7175 50  0000 C CNN "Component Value"
	1    4150 7275
	0    -1   -1   0   
$EndComp
$Comp
L 5.6pF_0402_04023J5R6BBWTR C210
U 1 1 6220405D
P 9300 7275
F 0 "C210" H 9350 7375 50  0000 L CNN
F 1 "5.6pF_0402_04023J5R6BBWTR" H 8700 7725 50  0001 L CNN
F 2 "SMD0402" H 9300 7625 47  0001 C CNN
F 3 "" H 9350 7375 60  0000 C CNN
F 4 "Mouser" H 9300 8225 60  0001 C CNN "Vendor"
F 5 "581-04023J5R6BBW" H 9300 8125 60  0001 C CNN "Vendor Part Number"
F 6 "AVX" H 9300 8025 60  0001 C CNN "Manufacturer"
F 7 "04023J5R6BBWTR" H 9300 7925 60  0001 C CNN "Manufacturer Part Number"
F 8 "Silicon RF Capacitors / Thin Film 25V 5.6pF .1pFTol ThinFilm 0402" H 9200 7825 60  0001 C CNN "Description"
F 9 "5.6pF" H 9425 7175 50  0000 C CNN "Component Value"
	1    9300 7275
	0    -1   -1   0   
$EndComp
$Comp
L 5.6pF_0402_04023J5R6BBWTR C194
U 1 1 6220406A
P 5625 5875
F 0 "C194" H 5675 5975 50  0000 L CNN
F 1 "5.6pF_0402_04023J5R6BBWTR" H 5025 6325 50  0001 L CNN
F 2 "SMD0402" H 5625 6225 47  0001 C CNN
F 3 "" H 5675 5975 60  0000 C CNN
F 4 "Mouser" H 5625 6825 60  0001 C CNN "Vendor"
F 5 "581-04023J5R6BBW" H 5625 6725 60  0001 C CNN "Vendor Part Number"
F 6 "AVX" H 5625 6625 60  0001 C CNN "Manufacturer"
F 7 "04023J5R6BBWTR" H 5625 6525 60  0001 C CNN "Manufacturer Part Number"
F 8 "Silicon RF Capacitors / Thin Film 25V 5.6pF .1pFTol ThinFilm 0402" H 5525 6425 60  0001 C CNN "Description"
F 9 "5.6pF" H 5750 5775 50  0000 C CNN "Component Value"
	1    5625 5875
	-1   0    0    1   
$EndComp
$Comp
L 5.6pF_0402_04023J5R6BBWTR C198
U 1 1 62204077
P 6375 4925
F 0 "C198" H 6425 5025 50  0000 L CNN
F 1 "5.6pF_0402_04023J5R6BBWTR" H 5775 5375 50  0001 L CNN
F 2 "SMD0402" H 6375 5275 47  0001 C CNN
F 3 "" H 6425 5025 60  0000 C CNN
F 4 "Mouser" H 6375 5875 60  0001 C CNN "Vendor"
F 5 "581-04023J5R6BBW" H 6375 5775 60  0001 C CNN "Vendor Part Number"
F 6 "AVX" H 6375 5675 60  0001 C CNN "Manufacturer"
F 7 "04023J5R6BBWTR" H 6375 5575 60  0001 C CNN "Manufacturer Part Number"
F 8 "Silicon RF Capacitors / Thin Film 25V 5.6pF .1pFTol ThinFilm 0402" H 6275 5475 60  0001 C CNN "Description"
F 9 "5.6pF" H 6500 4825 50  0000 C CNN "Component Value"
	1    6375 4925
	0    -1   -1   0   
$EndComp
$Comp
L 0.6pF_0402_GRM1555C2AR60BA01D C209
U 1 1 62204084
P 8625 7675
F 0 "C209" H 8675 7775 50  0000 L CNN
F 1 "0.6pF_0402_GRM1555C2AR60BA01D" H 8025 8575 50  0001 L CNN
F 2 "SMD0402" H 8625 8475 47  0001 C CNN
F 3 "" H 8675 7775 60  0000 C CNN
F 4 "Mouser" H 8625 9075 60  0001 C CNN "Vendor"
F 5 "81-GRM1555C2AR60BA1D" H 8625 8975 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 8625 8875 60  0001 C CNN "Manufacturer"
F 7 "GRM1555C2AR60BA01D" H 8625 8775 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 0402 0.6pF 100volts C0G +/-0.1pF" H 8525 8675 60  0001 C CNN "Description"
F 9 "0.6pF" H 8750 7575 50  0000 C CNN "Component Value"
	1    8625 7675
	1    0    0    -1  
$EndComp
$Comp
L 2pF_0402_GRM1555C2A2R0CA01D C208
U 1 1 62204091
P 7875 7675
F 0 "C208" H 7925 7775 50  0000 L CNN
F 1 "2pF_0402_GRM1555C2A2R0CA01D" H 7275 8050 50  0001 L CNN
F 2 "SMD0402" H 7875 7950 47  0001 C CNN
F 3 "" H 7925 7775 60  0000 C CNN
F 4 "Mouser" H 7875 8550 60  0001 C CNN "Vendor"
F 5 "81-GRM1555C2A2R0CA1D" H 7875 8450 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 7875 8350 60  0001 C CNN "Manufacturer"
F 7 "GRM1555C2A2R0CA01D" H 7875 8250 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 0402 2pF 100volts C0G +/-0.25pF" H 7775 8150 60  0001 C CNN "Description"
F 9 "2pF" H 8000 7575 50  0000 C CNN "Component Value"
	1    7875 7675
	1    0    0    -1  
$EndComp
$Comp
L 1uF_0603 C195
U 1 1 6220409E
P 5625 9475
F 0 "C195" H 5675 9575 50  0000 L CNN
F 1 "1uF_0603" H 5425 9925 50  0001 L CNN
F 2 "SMD0603" H 5625 9825 60  0001 C CNN
F 3 "" H 5625 9475 60  0000 C CNN
F 4 "Digi-Key" H 5625 10025 60  0001 C CNN "Vendor"
F 5 "490-3897-1-ND" H 5625 10125 60  0001 C CNN "Vendor Part Number"
F 6 "Murata" H 5625 10225 60  0001 C CNN "Manufacturer"
F 7 "GRM188R61E105KA12D" H 5625 10325 60  0001 C CNN "Manufacturer Part Number"
F 8 "CAP_CER_1UF_25V_X5R_0603" H 5625 10425 60  0001 C CNN "Description"
F 9 "1uF" H 5750 9375 50  0000 C CNN "Component Value"
	1    5625 9475
	1    0    0    -1  
$EndComp
$Comp
L 1uF_0603 C197
U 1 1 622040AB
P 6375 3925
F 0 "C197" H 6425 4025 50  0000 L CNN
F 1 "1uF_0603" H 6175 4375 50  0001 L CNN
F 2 "SMD0603" H 6375 4275 60  0001 C CNN
F 3 "" H 6375 3925 60  0000 C CNN
F 4 "Digi-Key" H 6375 4475 60  0001 C CNN "Vendor"
F 5 "490-3897-1-ND" H 6375 4575 60  0001 C CNN "Vendor Part Number"
F 6 "Murata" H 6375 4675 60  0001 C CNN "Manufacturer"
F 7 "GRM188R61E105KA12D" H 6375 4775 60  0001 C CNN "Manufacturer Part Number"
F 8 "CAP_CER_1UF_25V_X5R_0603" H 6375 4875 60  0001 C CNN "Description"
F 9 "1uF" H 6500 3825 50  0000 C CNN "Component Value"
	1    6375 3925
	0    -1   -1   0   
$EndComp
$Comp
L 1nF_0402 C202
U 1 1 622040B8
P 6750 9475
F 0 "C202" H 6800 9575 50  0000 L CNN
F 1 "1nF_0402" H 6950 9525 50  0001 L CNN
F 2 "SMD0402" H 6700 9475 60  0001 C CNN
F 3 "" H 6750 9475 60  0000 C CNN
F 4 "DigiKey" H 6850 9975 60  0001 C CNN "Vendor"
F 5 "311-1036-1-ND" H 6900 10075 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" H 6800 9875 60  0001 C CNN "Manufacturer"
F 7 "CC0402KRX7R9BB102" H 6850 9775 60  0001 C CNN "Manufacturer Part Number"
F 8 "CAP CER 1000PF 50V 10% X7R 0402" H 6950 9675 60  0001 C CNN "Description"
F 9 "1nF" H 6875 9375 50  0000 C CNN "Component Value"
	1    6750 9475
	1    0    0    -1  
$EndComp
$Comp
L 1nF_0402 C192
U 1 1 622040C5
P 4875 3450
F 0 "C192" H 4925 3550 50  0000 L CNN
F 1 "1nF_0402" H 5075 3500 50  0001 L CNN
F 2 "SMD0402" H 4825 3450 60  0001 C CNN
F 3 "" H 4875 3450 60  0000 C CNN
F 4 "DigiKey" H 4975 3950 60  0001 C CNN "Vendor"
F 5 "311-1036-1-ND" H 5025 4050 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" H 4925 3850 60  0001 C CNN "Manufacturer"
F 7 "CC0402KRX7R9BB102" H 4975 3750 60  0001 C CNN "Manufacturer Part Number"
F 8 "CAP CER 1000PF 50V 10% X7R 0402" H 5075 3650 60  0001 C CNN "Description"
F 9 "1nF" H 5000 3350 50  0000 C CNN "Component Value"
	1    4875 3450
	0    -1   -1   0   
$EndComp
$Comp
L 1nF_0402 C207
U 1 1 622040D2
P 7575 5325
F 0 "C207" H 7625 5425 50  0000 L CNN
F 1 "1nF_0402" H 7775 5375 50  0001 L CNN
F 2 "SMD0402" H 7525 5325 60  0001 C CNN
F 3 "" H 7575 5325 60  0000 C CNN
F 4 "DigiKey" H 7675 5825 60  0001 C CNN "Vendor"
F 5 "311-1036-1-ND" H 7725 5925 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" H 7625 5725 60  0001 C CNN "Manufacturer"
F 7 "CC0402KRX7R9BB102" H 7675 5625 60  0001 C CNN "Manufacturer Part Number"
F 8 "CAP CER 1000PF 50V 10% X7R 0402" H 7775 5525 60  0001 C CNN "Description"
F 9 "1nF" H 7700 5225 50  0000 C CNN "Component Value"
	1    7575 5325
	0    -1   -1   0   
$EndComp
$Comp
L 1nF_0402 C196
U 1 1 622040DF
P 6375 3450
F 0 "C196" H 6425 3550 50  0000 L CNN
F 1 "1nF_0402" H 6575 3500 50  0001 L CNN
F 2 "SMD0402" H 6325 3450 60  0001 C CNN
F 3 "" H 6375 3450 60  0000 C CNN
F 4 "DigiKey" H 6475 3950 60  0001 C CNN "Vendor"
F 5 "311-1036-1-ND" H 6525 4050 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" H 6425 3850 60  0001 C CNN "Manufacturer"
F 7 "CC0402KRX7R9BB102" H 6475 3750 60  0001 C CNN "Manufacturer Part Number"
F 8 "CAP CER 1000PF 50V 10% X7R 0402" H 6575 3650 60  0001 C CNN "Description"
F 9 "1nF" H 6500 3350 50  0000 C CNN "Component Value"
	1    6375 3450
	0    -1   -1   0   
$EndComp
$Comp
L 4.7uF_0603 C204
U 1 1 622040EC
P 7575 4850
F 0 "C204" H 7625 4950 50  0000 L CNN
F 1 "4.7uF_0603" H 7375 5200 50  0001 L CNN
F 2 "SMD0603" H 7575 5100 60  0001 C CNN
F 3 "" H 7575 4850 60  0000 C CNN
F 4 "DigiKey" H 7575 5600 60  0001 C CNN "Vendor"
F 5 "-" H 7575 5700 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 7575 5500 60  0001 C CNN "Manufacturer"
F 7 "-" H 7575 5400 60  0001 C CNN "Manufacturer Part Number"
F 8 "CAP_CER_4.7UF_?V_0603" H 7575 5300 60  0001 C CNN "Description"
F 9 "4.7uF" H 7700 4750 50  0000 C CNN "Component Value"
	1    7575 4850
	0    -1   -1   0   
$EndComp
$Comp
L 470pF_0402_GRM1555C1H471JA01D C199
U 1 1 622040F9
P 6625 5875
F 0 "C199" H 6675 5975 50  0000 L CNN
F 1 "470pF_0402_GRM1555C1H471JA01D" H 6025 6325 50  0001 L CNN
F 2 "SMD0402" H 6625 6225 47  0001 C CNN
F 3 "" H 6675 5975 60  0000 C CNN
F 4 "Mouser" H 6625 6825 60  0001 C CNN "Vendor"
F 5 "81-GRM1555C1H471JA01" H 6625 6725 60  0001 C CNN "Vendor Part Number"
F 6 "Murata Electronics" H 6625 6625 60  0001 C CNN "Manufacturer"
F 7 "GRM1555C1H471JA01D" H 6625 6525 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 0402 470pF 50volts C0G 5%" H 6525 6425 60  0001 C CNN "Description"
F 9 "470pF" H 6750 5775 50  0000 C CNN "Component Value"
	1    6625 5875
	0    1    1    0   
$EndComp
$Comp
L 10nF_0603_C0603C103J5RAC C193
U 1 1 62204106
P 4875 3925
F 0 "C193" H 4925 4025 50  0000 L CNN
F 1 "10nF_0603_C0603C103J5RAC" H 4900 4375 50  0001 C CNN
F 2 "SMD0603" H 4875 4275 60  0001 C CNN
F 3 "" H 4875 3925 60  0000 C CNN
F 4 "Mouser" H 4875 4875 60  0001 C CNN "Vendor"
F 5 "80-C0603C103J5RAC" H 4825 4775 60  0001 C CNN "Vendor Part Number"
F 6 "KEMET" H 4875 4675 60  0001 C CNN "Manufacturer"
F 7 "C0603C103J5RAC" H 4875 4575 60  0001 C CNN "Manufacturer Part Number"
F 8 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 50V 0.01uF 0603 X7R 0.05" H 4875 4475 60  0001 C CNN "Description"
F 9 "10nF" H 5000 3825 50  0000 C CNN "Component Value"
	1    4875 3925
	0    -1   -1   0   
$EndComp
$Comp
L 10nH_0603_0603HP-10NXGLW L14
U 1 1 62204113
P 5225 4325
F 0 "L14" V 5140 4355 50  0000 C CNN
F 1 "10nH_0603_0603HP-10NXGLW" H 5225 4575 50  0001 C CNN
F 2 "SMD0603" H 5225 4675 60  0001 C CNN
F 3 "" H 5225 4325 60  0000 C CNN
F 4 "Mouser" H 5225 5175 60  0001 C CNN "Vendor"
F 5 "994-0603HP-10NXGLW" H 5225 5075 60  0001 C CNN "Vendor Part Number"
F 6 "Coilcraft" H 5225 4975 60  0001 C CNN "Manufacturer"
F 7 "0603HP-10NXGLW" H 5225 4875 60  0001 C CNN "Manufacturer Part Number"
F 8 "Fixed Inductors 0603HP Hi Perfrmnce 10 nH 2 % 1.4 A" H 5225 4775 60  0001 C CNN "Description"
F 9 "10nH" V 5290 4305 50  0000 C CNN "Component Value"
	1    5225 4325
	1    0    0    -1  
$EndComp
$Comp
L 10nH_0603_0603HP-10NXGLW L16
U 1 1 62204120
P 6025 4325
F 0 "L16" V 5940 4355 50  0000 C CNN
F 1 "10nH_0603_0603HP-10NXGLW" H 6025 4575 50  0001 C CNN
F 2 "SMD0603" H 6025 4675 60  0001 C CNN
F 3 "" H 6025 4325 60  0000 C CNN
F 4 "Mouser" H 6025 5175 60  0001 C CNN "Vendor"
F 5 "994-0603HP-10NXGLW" H 6025 5075 60  0001 C CNN "Vendor Part Number"
F 6 "Coilcraft" H 6025 4975 60  0001 C CNN "Manufacturer"
F 7 "0603HP-10NXGLW" H 6025 4875 60  0001 C CNN "Manufacturer Part Number"
F 8 "Fixed Inductors 0603HP Hi Perfrmnce 10 nH 2 % 1.4 A" H 6025 4775 60  0001 C CNN "Description"
F 9 "10nH" V 6090 4305 50  0000 C CNN "Component Value"
	1    6025 4325
	1    0    0    -1  
$EndComp
$Comp
L 3.3nH_0603_0603HP-3N3XJLW L17
U 1 1 6220412D
P 7225 5725
F 0 "L17" V 7140 5755 50  0000 C CNN
F 1 "3.3nH_0603_0603HP-3N3XJLW" H 7225 5975 50  0001 C CNN
F 2 "SMD0603" H 7225 6075 60  0001 C CNN
F 3 "" H 7225 5725 60  0000 C CNN
F 4 "Mouser" H 7225 6575 60  0001 C CNN "Vendor"
F 5 "994-0603HP-3N3XJLW" H 7225 6475 60  0001 C CNN "Vendor Part Number"
F 6 "Coilcraft" H 7225 6375 60  0001 C CNN "Manufacturer"
F 7 "0603HP-3N3XJLW" H 7225 6275 60  0001 C CNN "Manufacturer Part Number"
F 8 "Fixed Inductors 0603 3.3nH Unshld 5% 1.9A 24mOhms" H 7225 6175 60  0001 C CNN "Description"
F 9 "3.3nH" V 7290 5705 50  0000 C CNN "Component Value"
	1    7225 5725
	1    0    0    -1  
$EndComp
$Comp
L 2.4nH_0402_0402CS-2N4XGLU L15
U 1 1 6220413A
P 5625 5200
F 0 "L15" V 5540 5230 50  0000 C CNN
F 1 "2.4nH_0402_0402CS-2N4XGLU" H 5625 5450 50  0001 C CNN
F 2 "SMD0402" H 5625 5550 60  0001 C CNN
F 3 "" H 5625 5200 60  0000 C CNN
F 4 "Mouser" H 5625 6050 60  0001 C CNN "Vendor"
F 5 "994-0402CS-2N4XGLU" H 5625 5950 60  0001 C CNN "Vendor Part Number"
F 6 "Coilcraft" H 5625 5850 60  0001 C CNN "Manufacturer"
F 7 "0402CS-2N4XGLU" H 5625 5750 60  0001 C CNN "Manufacturer Part Number"
F 8 "Fixed Inductors 1005 2.4nH Unshld 2% 790mA 68mOhms AECQ2" H 5625 5650 60  0001 C CNN "Description"
F 9 "2.4nH" V 5690 5180 50  0000 C CNN "Component Value"
	1    5625 5200
	1    0    0    -1  
$EndComp
$Comp
L 1.2K_0402_AC0402JR-071K2L R82
U 1 1 62204154
P 5425 8475
F 0 "R82" H 5425 8405 50  0000 C CNN
F 1 "1.2K_0402_AC0402JR-071K2L" H 5400 8725 50  0001 C CNN
F 2 "SMD0402" H 5400 8625 60  0001 C CNN
F 3 "" V 5425 8475 60  0000 C CNN
F 4 "Mouser" H 5350 9225 60  0001 C CNN "Vendor"
F 5 "603-AC0402JR-071K2L" H 5350 9125 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" H 5350 9025 60  0001 C CNN "Manufacturer"
F 7 "AC0402JR-071K2L" H 5350 8925 60  0001 C CNN "Manufacturer Part Number"
F 8 "Thick Film Resistors - SMD 1/16W 1.2K ohm 5% AEC-Q200" H 5350 8825 60  0001 C CNN "Description"
F 9 "1.2K" H 5425 8485 50  0000 C CNN "Component Value"
	1    5425 8475
	0    1    1    0   
$EndComp
$Comp
L 390R_0402_RC0402JR-07390RL R86
U 1 1 62204161
P 5625 8475
F 0 "R86" H 5625 8405 50  0000 C CNN
F 1 "390R_0402_RC0402JR-07390RL" H 5600 8725 50  0001 C CNN
F 2 "SMD0402" H 5600 8625 60  0001 C CNN
F 3 "" V 5625 8475 60  0000 C CNN
F 4 "Mouser" H 5625 9225 60  0001 C CNN "Vendor"
F 5 "603-RC0402JR-07390RL" H 5625 9125 60  0001 C CNN "Vendor Part Number"
F 6 "Yageo" H 5625 9025 60  0001 C CNN "Manufacturer"
F 7 "RC0402JR-07390RL" H 5625 8925 60  0001 C CNN "Manufacturer Part Number"
F 8 "Thick Film Resistors - SMD 390 OHM 5%" H 5550 8825 60  0001 C CNN "Description"
F 9 "390R" H 5625 8485 50  0000 C CNN "Component Value"
	1    5625 8475
	0    1    1    0   
$EndComp
$Comp
L MMZ38333BT1 U58
U 1 1 6220416E
P 5725 7175
F 0 "U58" H 5725 7225 60  0000 C CNN
F 1 "MMZ38333BT1" H 5725 7125 60  0000 C CNN
F 2 "MMZ38333BT1" H 5725 8300 60  0001 C CNN
F 3 "" H 5725 8300 60  0000 C CNN
F 4 "Mouser" H 5725 8800 60  0001 C CNN "Vendor"
F 5 "841-MMZ38333BT1" H 5725 8700 60  0001 C CNN "Vendor Part Number"
F 6 "NXP Semiconductors" H 5725 8600 60  0001 C CNN "Manufacturer"
F 7 "MMZ38333BT1" H 5725 8500 60  0001 C CNN "Manufacturer Part Number"
F 8 "RF Amplifier InGaP HBT Linear Amplifier, 3400-3800 MHz, 37 dB, 32 dBm." H 5725 8400 60  0001 C CNN "Description"
F 9 "MMZ38333BT1" H 5725 8200 60  0001 C CNN "Component Value"
	1    5725 7175
	1    0    0    -1  
$EndComp
$Comp
L 0RNF_0402 R87
U 1 1 6220417B
P 6575 5500
F 0 "R87" H 6565 5570 50  0000 C CNN
F 1 "0RNF_0402" H 6575 5650 50  0001 C CNN
F 2 "SMD0402" H 6575 5750 60  0001 C CNN
F 3 "" V 6575 5500 60  0000 C CNN
F 4 "Digi-Key" H 6575 6150 60  0001 C CNN "Vendor"
F 5 "-" H 6575 6250 60  0001 C CNN "Vendor Part Number"
F 6 "-" H 6575 6050 60  0001 C CNN "Manufacturer"
F 7 "-" H 6575 5950 60  0001 C CNN "Manufacturer Part Number"
F 8 "RES_0.0_OHM_0402_SMD" H 6575 5850 60  0001 C CNN "Description"
F 9 "NF" H 6575 5490 50  0000 C CNN "Component Value"
	1    6575 5500
	1    0    0    -1  
$EndComp
Text Label 5625 5575 0    60   ~ 0
z5
Text Notes 1150 8550 0    120  ~ 24
Vbias=5V
$Comp
L SN74LV1T34 U59
U 1 1 62204196
P 7725 9025
F 0 "U59" H 7925 8825 60  0000 C CNN
F 1 "SN74LV1T34" H 7725 9225 60  0000 C CNN
F 2 "SN74LV1T34" H 7775 9625 60  0001 C CNN
F 3 "" H 7525 9025 60  0000 C CNN
F 4 "Mouser" H 7775 10150 60  0001 C CNN "Vendor"
F 5 "595-SN74LV1T34DCKRG4" H 7775 10050 60  0001 C CNN "Vendor Part Number"
F 6 "SN74LV1T34DCKRG4" H 7775 9850 60  0001 C CNN "Manufacturer"
F 7 "Texas Instruments" H 7775 9950 60  0001 C CNN "Manufacturer Part Number"
F 8 "Translation - Voltage Levels Single Power Supply BUFFER Logic Level Shifter" H 7775 9750 60  0001 C CNN "Description"
F 9 "BUFF_SN74LV1T34" H 7675 9475 60  0001 C CNN "Component Value"
	1    7725 9025
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6725 3925 6575 3925
Wire Wire Line
	6725 4925 6575 4925
Wire Wire Line
	6025 3925 6175 3925
Wire Wire Line
	6025 4925 6175 4925
Connection ~ 6025 4925
Wire Wire Line
	7225 5925 7225 7275
Connection ~ 7225 7275
Connection ~ 6025 3925
Wire Wire Line
	7925 2625 7725 2625
Wire Wire Line
	5225 2625 7425 2625
Connection ~ 7225 2625
Wire Wire Line
	6675 7275 9100 7275
Wire Wire Line
	6725 3450 6575 3450
Wire Wire Line
	8625 8025 8625 7875
Wire Wire Line
	6175 3450 6025 3450
Connection ~ 6025 3450
Wire Wire Line
	6675 7075 7225 7075
Connection ~ 7225 7075
Wire Wire Line
	6675 6875 7225 6875
Connection ~ 7225 6875
Wire Wire Line
	3650 7275 3950 7275
Wire Wire Line
	4350 7275 4775 7275
Wire Wire Line
	6750 9825 6750 9675
Wire Wire Line
	5625 9825 5625 9675
Wire Wire Line
	5625 8725 5625 9275
Connection ~ 5225 4925
Wire Wire Line
	5225 4925 5625 4925
Wire Wire Line
	5625 5400 5625 5675
Wire Wire Line
	5625 4925 5625 5000
Wire Wire Line
	5625 6075 5625 6225
Wire Wire Line
	6900 5875 6825 5875
Wire Wire Line
	6025 4525 6025 6225
Wire Wire Line
	7925 5325 7775 5325
Wire Wire Line
	7225 5325 7375 5325
Connection ~ 7225 5325
Wire Wire Line
	7925 4850 7775 4850
Wire Wire Line
	7225 4850 7375 4850
Connection ~ 7225 4850
Wire Wire Line
	6025 2625 6025 4125
Wire Wire Line
	7225 2625 7225 5525
Wire Wire Line
	4525 3925 4675 3925
Wire Wire Line
	5225 3925 5075 3925
Connection ~ 5225 3925
Wire Wire Line
	4525 3450 4675 3450
Wire Wire Line
	5075 3450 5225 3450
Connection ~ 5225 3450
Connection ~ 6025 2625
Wire Wire Line
	5225 4525 5225 6225
Wire Wire Line
	5225 3275 5225 4125
Wire Wire Line
	5225 2775 5225 2625
Wire Wire Line
	7875 8025 7875 7875
Wire Wire Line
	8125 9025 9825 9025
Wire Wire Line
	5425 8925 5425 8725
Wire Wire Line
	5425 8225 5425 8125
Wire Wire Line
	5625 8225 5625 8125
Wire Wire Line
	5825 8125 5825 9125
Wire Wire Line
	6225 5875 6425 5875
Wire Wire Line
	6225 5500 6225 6225
Wire Wire Line
	6900 5500 6825 5500
Wire Wire Line
	6325 5500 6225 5500
Connection ~ 6225 5875
Wire Wire Line
	1800 9025 1950 9025
Wire Wire Line
	1800 8925 7325 8925
Connection ~ 5625 8925
Connection ~ 5425 8925
Wire Wire Line
	5825 9125 7325 9125
Wire Wire Line
	6750 9275 6750 9125
Connection ~ 6750 9125
Wire Wire Line
	8275 9125 8125 9125
NoConn ~ 8125 8925
Text Notes 11175 1700 0    118  ~ 0
Datasheet Substrate: \nRogers4350B, Er=3.66, tickness = 254 um (10mil)\nPCB was modified for substrate:\nRogers4350B, Er=3.66, tickness = 508 um (20mil)\n
Text HLabel 2250 7275 0    60   Input ~ 0
RF_in
Wire Wire Line
	2250 7275 3250 7275
Text HLabel 10550 7275 2    60   Output ~ 0
RF_out
Wire Wire Line
	10550 7275 9500 7275
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01328
U 1 1 622041ED
P 7925 4850
F 0 "#PWR01328" H 7925 4950 50  0001 C CNN
F 1 "GND" H 7925 4790 50  0000 C CNN
F 2 "" H 7925 4850 60  0001 C CNN
F 3 "" H 7925 4850 60  0001 C CNN
	1    7925 4850
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01329
U 1 1 622041F3
P 7925 5325
F 0 "#PWR01329" H 7925 5425 50  0001 C CNN
F 1 "GND" H 7925 5265 50  0000 C CNN
F 2 "" H 7925 5325 60  0001 C CNN
F 3 "" H 7925 5325 60  0001 C CNN
	1    7925 5325
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01330
U 1 1 622041F9
P 4525 3450
F 0 "#PWR01330" H 4525 3550 50  0001 C CNN
F 1 "GND" H 4525 3390 50  0000 C CNN
F 2 "" H 4525 3450 60  0001 C CNN
F 3 "" H 4525 3450 60  0001 C CNN
	1    4525 3450
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01331
U 1 1 622041FF
P 4525 3925
F 0 "#PWR01331" H 4525 4025 50  0001 C CNN
F 1 "GND" H 4525 3865 50  0000 C CNN
F 2 "" H 4525 3925 60  0001 C CNN
F 3 "" H 4525 3925 60  0001 C CNN
	1    4525 3925
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01332
U 1 1 62204205
P 6900 5875
F 0 "#PWR01332" H 6900 5975 50  0001 C CNN
F 1 "GND" H 6900 5815 50  0000 C CNN
F 2 "" H 6900 5875 60  0001 C CNN
F 3 "" H 6900 5875 60  0001 C CNN
	1    6900 5875
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01333
U 1 1 6220420B
P 6900 5500
F 0 "#PWR01333" H 6900 5600 50  0001 C CNN
F 1 "GND" H 6900 5440 50  0000 C CNN
F 2 "" H 6900 5500 60  0001 C CNN
F 3 "" H 6900 5500 60  0001 C CNN
	1    6900 5500
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01334
U 1 1 62204211
P 6725 4925
F 0 "#PWR01334" H 6725 5025 50  0001 C CNN
F 1 "GND" H 6725 4865 50  0000 C CNN
F 2 "" H 6725 4925 60  0001 C CNN
F 3 "" H 6725 4925 60  0001 C CNN
	1    6725 4925
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01335
U 1 1 62204217
P 6725 3925
F 0 "#PWR01335" H 6725 4025 50  0001 C CNN
F 1 "GND" H 6725 3865 50  0000 C CNN
F 2 "" H 6725 3925 60  0001 C CNN
F 3 "" H 6725 3925 60  0001 C CNN
	1    6725 3925
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01336
U 1 1 6220421D
P 6725 3450
F 0 "#PWR01336" H 6725 3550 50  0001 C CNN
F 1 "GND" H 6725 3390 50  0000 C CNN
F 2 "" H 6725 3450 60  0001 C CNN
F 3 "" H 6725 3450 60  0001 C CNN
	1    6725 3450
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01337
U 1 1 62204223
P 5625 9825
F 0 "#PWR01337" H 5625 9925 50  0001 C CNN
F 1 "GND" H 5625 9765 50  0000 C CNN
F 2 "" H 5625 9825 60  0001 C CNN
F 3 "" H 5625 9825 60  0001 C CNN
	1    5625 9825
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01338
U 1 1 62204229
P 6750 9825
F 0 "#PWR01338" H 6750 9925 50  0001 C CNN
F 1 "GND" H 6750 9765 50  0000 C CNN
F 2 "" H 6750 9825 60  0001 C CNN
F 3 "" H 6750 9825 60  0001 C CNN
	1    6750 9825
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01339
U 1 1 6220422F
P 8275 9125
F 0 "#PWR01339" H 8275 9225 50  0001 C CNN
F 1 "GND" H 8275 9065 50  0000 C CNN
F 2 "" H 8275 9125 60  0001 C CNN
F 3 "" H 8275 9125 60  0001 C CNN
	1    8275 9125
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01340
U 1 1 62204235
P 9675 9125
F 0 "#PWR01340" H 9675 9225 50  0001 C CNN
F 1 "GND" H 9675 9065 50  0000 C CNN
F 2 "" H 9675 9125 60  0001 C CNN
F 3 "" H 9675 9125 60  0001 C CNN
	1    9675 9125
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01341
U 1 1 62204241
P 1950 9025
F 0 "#PWR01341" H 1950 9125 50  0001 C CNN
F 1 "GND" H 1950 8965 50  0000 C CNN
F 2 "" H 1950 9025 60  0001 C CNN
F 3 "" H 1950 9025 60  0001 C CNN
	1    1950 9025
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01342
U 1 1 6220424D
P 6850 8050
F 0 "#PWR01342" H 6850 8150 50  0001 C CNN
F 1 "GND" H 6850 7990 50  0000 C CNN
F 2 "" H 6850 8050 60  0001 C CNN
F 3 "" H 6850 8050 60  0001 C CNN
	1    6850 8050
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01343
U 1 1 62204253
P 6850 7675
F 0 "#PWR01343" H 6850 7775 50  0001 C CNN
F 1 "GND" H 6850 7615 50  0000 C CNN
F 2 "" H 6850 7675 60  0001 C CNN
F 3 "" H 6850 7675 60  0001 C CNN
	1    6850 7675
	0    -1   -1   0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01344
U 1 1 62204259
P 6850 7475
F 0 "#PWR01344" H 6850 7575 50  0001 C CNN
F 1 "GND" H 6850 7415 50  0000 C CNN
F 2 "" H 6850 7475 60  0001 C CNN
F 3 "" H 6850 7475 60  0001 C CNN
	1    6850 7475
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6850 7675 6675 7675
Wire Wire Line
	6850 8050 6675 8050
Wire Wire Line
	6850 7475 6675 7475
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01345
U 1 1 62204262
P 4575 7675
F 0 "#PWR01345" H 4575 7775 50  0001 C CNN
F 1 "GND" H 4575 7615 50  0000 C CNN
F 2 "" H 4575 7675 60  0001 C CNN
F 3 "" H 4575 7675 60  0001 C CNN
	1    4575 7675
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01346
U 1 1 62204268
P 4550 6875
F 0 "#PWR01346" H 4550 6975 50  0001 C CNN
F 1 "GND" H 4550 6815 50  0000 C CNN
F 2 "" H 4550 6875 60  0001 C CNN
F 3 "" H 4550 6875 60  0001 C CNN
	1    4550 6875
	0    1    1    0   
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01347
U 1 1 6220426E
P 4550 6675
F 0 "#PWR01347" H 4550 6775 50  0001 C CNN
F 1 "GND" H 4550 6615 50  0000 C CNN
F 2 "" H 4550 6675 60  0001 C CNN
F 3 "" H 4550 6675 60  0001 C CNN
	1    4550 6675
	0    1    1    0   
$EndComp
Wire Wire Line
	4575 7675 4775 7675
Wire Wire Line
	4550 6875 4775 6875
Wire Wire Line
	4775 6675 4550 6675
$Comp
L +5V_RF #PWR01348
U 1 1 62204277
P 6025 1975
F 0 "#PWR01348" H 6025 2025 20  0001 C CNN
F 1 "+5V_RF" H 6025 2075 30  0000 C CNN
F 2 "" H 6025 1975 60  0000 C CNN
F 3 "" H 6025 1975 60  0000 C CNN
	1    6025 1975
	1    0    0    -1  
$EndComp
Connection ~ 6575 2625
Wire Wire Line
	6025 2275 6575 2275
Wire Wire Line
	6025 1975 6025 2275
$Comp
L +5V_RF #PWR01349
U 1 1 62204281
P 2875 8575
F 0 "#PWR01349" H 2875 8625 20  0001 C CNN
F 1 "+5V_RF" H 2875 8675 30  0000 C CNN
F 2 "" H 2875 8575 60  0000 C CNN
F 3 "" H 2875 8575 60  0000 C CNN
	1    2875 8575
	1    0    0    -1  
$EndComp
Wire Wire Line
	2875 8575 2875 8925
Connection ~ 2875 8925
Wire Wire Line
	9675 9125 9825 9125
Text HLabel 9925 10625 2    60   Input ~ 0
EN
Wire Wire Line
	9925 10625 8925 10625
Wire Wire Line
	8925 10625 8925 9025
Connection ~ 8925 9025
Wire Wire Line
	7875 7475 7875 7275
Connection ~ 7875 7275
Wire Wire Line
	8625 7475 8625 7275
Connection ~ 8625 7275
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01350
U 1 1 62204292
P 7875 8025
F 0 "#PWR01350" H 7875 8125 50  0001 C CNN
F 1 "GND" H 7875 7965 50  0000 C CNN
F 2 "" H 7875 8025 60  0001 C CNN
F 3 "" H 7875 8025 60  0001 C CNN
	1    7875 8025
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01351
U 1 1 62204298
P 8625 8025
F 0 "#PWR01351" H 8625 8125 50  0001 C CNN
F 1 "GND" H 8625 7965 50  0000 C CNN
F 2 "" H 8625 8025 60  0001 C CNN
F 3 "" H 8625 8025 60  0001 C CNN
	1    8625 8025
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-LMS8001_AppPCB #PWR01352
U 1 1 6220429E
P 7925 2625
F 0 "#PWR01352" H 7925 2725 50  0001 C CNN
F 1 "GND" H 7925 2565 50  0000 C CNN
F 2 "" H 7925 2625 60  0001 C CNN
F 3 "" H 7925 2625 60  0001 C CNN
	1    7925 2625
	0    -1   -1   0   
$EndComp
$Comp
L 27R_0402_ERJ2RKD27R0X R81
U 1 1 622B1ADE
P 5225 3025
F 0 "R81" H 5215 3095 50  0000 C CNN
F 1 "27R_0402_ERJ2RKD27R0X" H 5225 3275 61  0001 C CNN
F 2 "SMD0402" H 5225 3375 60  0001 C CNN
F 3 "" V 5225 3025 60  0000 C CNN
F 4 "Mouser" H 5225 3875 60  0001 C CNN "Vendor"
F 5 "667-ERJ-2RKD27R0X" H 5225 3775 60  0001 C CNN "Vendor Part Number"
F 6 "Panasonic" H 5225 3675 60  0001 C CNN "Manufacturer"
F 7 "ERJ2RKD27R0X" H 5225 3575 60  0001 C CNN "Manufacturer Part Number"
F 8 "Thick Film Resistors - SMD 0402 Resistor 0.5% 100ppm 27Ohm" H 5225 3475 60  0001 C CNN "Description"
F 9 "27R" H 5225 3023 50  0000 C CNN "Component Value"
	1    5225 3025
	0    1    1    0   
$EndComp
$Comp
L TSM-102-04-T-SV J16
U 1 1 626FB2C8
P 1550 8975
F 0 "J16" H 1550 9225 60  0000 C CNN
F 1 "TSM-102-04-T-SV" H 1550 8775 60  0000 C CNN
F 2 "TSM-102-04-T-SV" H 1550 9625 60  0001 C CNN
F 3 "" H 1550 8875 60  0000 C CNN
F 4 "Mouser" H 1550 10100 60  0001 C CNN "Vendor"
F 5 "TSM-102-04-T-SV" H 1550 9825 60  0001 C CNN "Vendor Part Number"
F 6 "Samtec" H 1550 9925 60  0001 C CNN "Manufacturer"
F 7 "200-TSM10204TSV" H 1550 10025 60  0001 C CNN "Manufacturer Part Number"
F 8 "Headers & Wire Housings .100\" Surface Mount Terminal Strip" H 1575 9725 60  0001 C CNN "Description"
F 9 "CONN_HEADER_TSM-102-04-T-SV" H 1550 9525 60  0001 C CNN "Component Value"
	1    1550 8975
	-1   0    0    1   
$EndComp
$Comp
L TSM-102-04-T-SV J18
U 1 1 626FF723
P 10075 9075
F 0 "J18" H 10075 9325 60  0000 C CNN
F 1 "TSM-102-04-T-SV" H 10075 8875 60  0000 C CNN
F 2 "TSM-102-04-T-SV" H 10075 9725 60  0001 C CNN
F 3 "" H 10075 8975 60  0000 C CNN
F 4 "Mouser" H 10075 10200 60  0001 C CNN "Vendor"
F 5 "TSM-102-04-T-SV" H 10075 9925 60  0001 C CNN "Vendor Part Number"
F 6 "Samtec" H 10075 10025 60  0001 C CNN "Manufacturer"
F 7 "200-TSM10204TSV" H 10075 10125 60  0001 C CNN "Manufacturer Part Number"
F 8 "Headers & Wire Housings .100\" Surface Mount Terminal Strip" H 10100 9825 60  0001 C CNN "Description"
F 9 "CONN_HEADER_TSM-102-04-T-SV" H 10075 9625 60  0001 C CNN "Component Value"
	1    10075 9075
	1    0    0    1   
$EndComp
Wire Wire Line
	6575 2275 6575 2625
$EndSCHEMATC
