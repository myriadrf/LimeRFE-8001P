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
Sheet 1 20
Title "FrontEnd_5G"
Date "10 Feb 2022"
Rev "1v0"
Comp "Lime Microsystems"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 900  900  1800 1000
U 55E9B74A
F0 "LMS8001 Power Supply" 50
F1 "LMS8001_PowerSupply.sch" 50
$EndSheet
$Sheet
S 900  2500 1800 1000
U 55ED9BB4
F0 "PowerSupply" 50
F1 "PowerSupply.sch" 50
$EndSheet
$Sheet
S 3300 900  1800 1000
U 55EDA5A1
F0 "Clocks" 50
F1 "Clocks.sch" 50
$EndSheet
Wire Wire Line
	3300 5300 3000 5300
Text Label 3000 5300 0    60   ~ 0
DATA2
Text Label 6450 5300 2    60   ~ 0
DATAX
$Sheet
S 3300 2500 1800 1000
U 55F3CC96
F0 "LMS8001_RF" 50
F1 "LMS8001_RF.sch" 50
F2 "DATA_IN" I L 3300 3350 59 
F3 "DATA_OUT" O R 5100 3350 59 
$EndSheet
$Sheet
S 5700 2500 1800 1000
U 62ABBA48
F0 "RP2040_MCU" 50
F1 "RP2040_MCU.sch" 50
F2 "DATA_OUT" O R 7500 2600 59 
$EndSheet
Wire Wire Line
	7500 2600 7800 2600
Text Label 7800 2600 2    60   ~ 0
DATA1
$Sheet
S 5700 900  1800 1000
U 6279CAB2
F0 "LMS98001_GPIO" 59
F1 "LMS98001_GPIO.sch" 59
$EndSheet
$Sheet
S 8100 900  1800 1000
U 625D43BC
F0 "SC1905_1" 50
F1 "SC1905_1.sch" 50
$EndSheet
$Sheet
S 8100 2500 1800 1000
U 625D4A2B
F0 "SC1905_2" 50
F1 "SC1905_2.sch" 50
$EndSheet
Wire Wire Line
	6150 5300 6450 5300
NoConn ~ 6450 5300
$Sheet
S 3300 4100 2850 1300
U 62333877
F0 "RF" 60
F1 "RF.sch" 60
F2 "DATA_IN" I L 3300 5300 60 
F3 "DATA_OUT" O R 6150 5300 60 
$EndSheet
Wire Wire Line
	3300 3350 3000 3350
Text Label 3000 3350 0    60   ~ 0
DATA1
Wire Wire Line
	5100 3350 5400 3350
Text Label 5400 3350 2    60   ~ 0
DATA2
$EndSCHEMATC
