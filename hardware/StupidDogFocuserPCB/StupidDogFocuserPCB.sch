EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "Stupid Dog Focuser"
Date "2020-02-16"
Rev "1"
Comp "Stupid Dog Observatory"
Comment1 "Jeff Voight"
Comment2 "Copyright 2020"
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	3250 2750 3350 2750
Wire Wire Line
	3350 2850 3250 2850
Wire Wire Line
	2600 6500 2650 6500
Wire Wire Line
	2600 6450 2600 6500
Text GLabel 2650 6500 2    50   Input ~ 0
GND
Connection ~ 2600 6100
Wire Wire Line
	2600 6150 2600 6100
$Comp
L Device:C C5
U 1 1 5E5521CB
P 2600 6300
F 0 "C5" H 2715 6346 50  0000 L CNN
F 1 "4.7uF" H 2715 6255 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 2638 6150 50  0001 C CNN
F 3 "~" H 2600 6300 50  0001 C CNN
	1    2600 6300
	1    0    0    -1  
$EndComp
Connection ~ 2750 3650
Wire Wire Line
	2650 3650 2750 3650
Wire Wire Line
	2650 3450 2650 3650
Wire Wire Line
	2750 3650 2850 3650
Wire Wire Line
	2750 3450 2750 3650
Wire Wire Line
	2850 3450 2850 3650
Wire Wire Line
	7250 5250 7350 5250
Text GLabel 7250 5250 0    50   Input ~ 0
GND
Text GLabel 2850 3650 2    50   Input ~ 0
GND
Wire Wire Line
	8050 5250 7950 5250
Text GLabel 8050 5250 2    50   Input ~ 0
VCC
Wire Wire Line
	5800 2000 5900 2000
Connection ~ 5800 2000
Wire Wire Line
	5800 2100 5800 2000
Wire Wire Line
	5900 2100 5800 2100
Wire Wire Line
	5750 2000 5800 2000
Text GLabel 5750 2000 0    50   Input ~ 0
VCC
Wire Wire Line
	6700 2600 6800 2600
Wire Wire Line
	6700 2500 6800 2500
Wire Wire Line
	6700 2300 6800 2300
Wire Wire Line
	6700 2200 6800 2200
Text GLabel 8650 2250 2    50   Input ~ 0
G
Text GLabel 8650 2350 2    50   Input ~ 0
BLK
Text GLabel 8650 2550 2    50   Input ~ 0
BL
Text GLabel 8650 2450 2    50   Input ~ 0
R
Text GLabel 6800 2600 2    50   Input ~ 0
BL
Text GLabel 6800 2500 2    50   Input ~ 0
R
Text GLabel 6800 2300 2    50   Input ~ 0
G
Text GLabel 6800 2200 2    50   Input ~ 0
BLK
Wire Wire Line
	5800 2700 5900 2700
Text GLabel 5800 2700 0    50   Input ~ 0
M0
Wire Wire Line
	5800 2300 5900 2300
Text GLabel 5800 2300 0    50   Input ~ 0
~Enable
Wire Wire Line
	5800 2500 5900 2500
Wire Wire Line
	5800 2400 5900 2400
Wire Wire Line
	2150 1950 2250 1950
Wire Wire Line
	2150 2050 2250 2050
Wire Wire Line
	2150 2250 2250 2250
Wire Wire Line
	2150 2350 2250 2350
Wire Wire Line
	2150 2450 2250 2450
Wire Wire Line
	7650 5700 7650 5550
Text GLabel 7650 5700 3    50   Input ~ 0
DHT
Text GLabel 2150 2950 0    50   Input ~ 0
DHT
Text GLabel 3350 2650 2    50   Input ~ 0
M0
Text GLabel 2150 2450 0    50   Input ~ 0
SW
Text GLabel 2150 2350 0    50   Input ~ 0
ENCB
Text GLabel 2150 2250 0    50   Input ~ 0
ENCA
Text GLabel 3350 2550 2    50   Input ~ 0
~Enable
Text GLabel 5800 2500 0    50   Input ~ 0
Dir
Text GLabel 5800 2400 0    50   Input ~ 0
Step
Text GLabel 2150 2050 0    50   Input ~ 0
Dir
Text GLabel 2150 1950 0    50   Input ~ 0
Step
NoConn ~ 2850 1350
Wire Wire Line
	2950 1350 2950 1200
Wire Wire Line
	2650 1350 2650 1200
Connection ~ 6400 3300
Wire Wire Line
	6300 3300 6300 3100
Wire Wire Line
	6400 3300 6300 3300
Wire Wire Line
	6400 3300 6400 3100
Wire Wire Line
	6450 3300 6400 3300
Wire Wire Line
	6300 1550 6300 1500
Connection ~ 6300 1550
Wire Wire Line
	6400 1550 6300 1550
Text GLabel 6400 1550 2    50   Input ~ 0
VDD
Wire Wire Line
	6300 1200 6350 1200
Wire Wire Line
	6300 1700 6300 1550
Wire Wire Line
	2150 5800 2300 5800
Wire Wire Line
	2150 5700 2300 5700
Wire Wire Line
	2150 5600 2300 5600
Text GLabel 2150 5600 0    50   Input ~ 0
ENCA
Text GLabel 2150 5700 0    50   Input ~ 0
ENCB
Text GLabel 2150 5800 0    50   Input ~ 0
SW
NoConn ~ 2300 5300
NoConn ~ 2300 5400
NoConn ~ 2900 5400
NoConn ~ 2900 5300
Wire Wire Line
	2900 5800 2950 5800
Wire Wire Line
	2900 5700 2950 5700
Wire Wire Line
	2900 5600 2950 5600
Text GLabel 2950 5800 2    50   Input ~ 0
SW_A
Text GLabel 2950 5700 2    50   Input ~ 0
ENCB_A
Text GLabel 2950 5600 2    50   Input ~ 0
ENCA_A
Wire Wire Line
	5400 5450 5550 5450
Wire Wire Line
	5800 5650 5850 5650
Connection ~ 5800 5650
Wire Wire Line
	5800 5500 5800 5650
Wire Wire Line
	5400 5650 5800 5650
Wire Wire Line
	5800 5150 5900 5150
Connection ~ 5800 5150
Wire Wire Line
	5800 5200 5800 5150
Wire Wire Line
	5550 5150 5800 5150
Text GLabel 5900 5150 2    50   Input ~ 0
SW_A
Connection ~ 4800 4800
Wire Wire Line
	5400 4800 4800 4800
Connection ~ 3850 5550
Wire Wire Line
	3850 6300 4800 6300
Wire Wire Line
	3850 5550 3850 6300
Wire Wire Line
	3850 4800 4800 4800
Wire Wire Line
	3850 5550 3850 4800
Wire Wire Line
	3750 5550 3850 5550
Wire Wire Line
	4350 5950 4550 5950
Connection ~ 4350 5950
Wire Wire Line
	4350 5850 4350 5950
Wire Wire Line
	4350 5150 4550 5150
Connection ~ 4350 5150
Wire Wire Line
	4350 5200 4350 5150
Wire Wire Line
	4350 5500 4350 5550
Wire Wire Line
	4350 5550 4800 5550
Connection ~ 4350 5550
Wire Wire Line
	4150 5550 4350 5550
Wire Wire Line
	4300 5150 4350 5150
Wire Wire Line
	4300 5950 4350 5950
Text GLabel 4300 5950 0    50   Input ~ 0
ENCB_A
Text GLabel 4300 5150 0    50   Input ~ 0
ENCA_A
Wire Wire Line
	4550 5650 4800 5650
Wire Wire Line
	4800 5450 4550 5450
Text GLabel 5850 5650 2    50   Input ~ 0
GND
Text GLabel 4150 5550 0    50   Input ~ 0
GND
Text GLabel 3750 5550 0    50   Input ~ 0
VCC
Wire Wire Line
	2600 6100 2700 6100
Text GLabel 2700 6100 2    50   Input ~ 0
VCC
Wire Wire Line
	2700 5000 2600 5000
Text GLabel 2700 5000 2    50   Input ~ 0
GND
Wire Wire Line
	9600 5500 9600 5650
Wire Wire Line
	9300 5500 9300 5650
Wire Wire Line
	9000 5500 9000 5650
$Comp
L power:+5V #PWR02
U 1 1 5E4B2D8D
P 9300 5500
F 0 "#PWR02" H 9300 5350 50  0001 C CNN
F 1 "+5V" H 9315 5673 50  0000 C CNN
F 2 "" H 9300 5500 50  0001 C CNN
F 3 "" H 9300 5500 50  0001 C CNN
	1    9300 5500
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR01
U 1 1 5E4B2215
P 9000 5500
F 0 "#PWR01" H 9000 5350 50  0001 C CNN
F 1 "+12V" H 9015 5673 50  0000 C CNN
F 2 "" H 9000 5500 50  0001 C CNN
F 3 "" H 9000 5500 50  0001 C CNN
	1    9000 5500
	1    0    0    -1  
$EndComp
Text GLabel 6350 1200 2    50   Input ~ 0
GND
Text GLabel 6450 3300 2    50   Input ~ 0
GND
Text GLabel 2950 1200 1    50   Input ~ 0
VCC
Text GLabel 2650 1200 1    50   Input ~ 0
VDD
Text GLabel 9000 5650 3    50   Input ~ 0
VDD
Text GLabel 9300 5650 3    50   Input ~ 0
VCC
Text GLabel 9600 5500 1    50   Input ~ 0
GND
$Comp
L power:GND #PWR03
U 1 1 5E4B0863
P 9600 5650
F 0 "#PWR03" H 9600 5400 50  0001 C CNN
F 1 "GND" H 9605 5477 50  0000 C CNN
F 2 "" H 9600 5650 50  0001 C CNN
F 3 "" H 9600 5650 50  0001 C CNN
	1    9600 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5E4AECAA
P 6300 1350
F 0 "C1" H 6415 1396 50  0000 L CNN
F 1 "4.7uF" H 6415 1305 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 6338 1200 50  0001 C CNN
F 3 "~" H 6300 1350 50  0001 C CNN
	1    6300 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5E4AB473
P 5400 4950
F 0 "R2" H 5470 4996 50  0000 L CNN
F 1 "1k" H 5470 4905 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 5330 4950 50  0001 C CNN
F 3 "~" H 5400 4950 50  0001 C CNN
	1    5400 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5E4AB0D6
P 5550 5300
F 0 "R4" H 5620 5346 50  0000 L CNN
F 1 "1k" H 5620 5255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 5480 5300 50  0001 C CNN
F 3 "~" H 5550 5300 50  0001 C CNN
	1    5550 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5E4AADCA
P 4800 6150
F 0 "R6" H 4870 6196 50  0000 L CNN
F 1 "1k" H 4870 6105 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4730 6150 50  0001 C CNN
F 3 "~" H 4800 6150 50  0001 C CNN
	1    4800 6150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5E4AAB06
P 4550 5800
F 0 "R5" H 4620 5846 50  0000 L CNN
F 1 "1k" H 4620 5755 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4480 5800 50  0001 C CNN
F 3 "~" H 4550 5800 50  0001 C CNN
	1    4550 5800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5E4AA80A
P 4550 5300
F 0 "R3" H 4620 5346 50  0000 L CNN
F 1 "1k" H 4620 5255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4480 5300 50  0001 C CNN
F 3 "~" H 4550 5300 50  0001 C CNN
	1    4550 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5E4A9F5B
P 4800 4950
F 0 "R1" H 4870 4996 50  0000 L CNN
F 1 "1k" H 4870 4905 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4730 4950 50  0001 C CNN
F 3 "~" H 4800 4950 50  0001 C CNN
	1    4800 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5E4A97CD
P 5800 5350
F 0 "C3" H 5915 5396 50  0000 L CNN
F 1 "4.7uF" H 5915 5305 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 5838 5200 50  0001 C CNN
F 3 "~" H 5800 5350 50  0001 C CNN
	1    5800 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5E4A9076
P 4350 5700
F 0 "C4" H 4200 5800 50  0000 L CNN
F 1 "4.7uF" H 4050 5700 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 4388 5550 50  0001 C CNN
F 3 "~" H 4350 5700 50  0001 C CNN
	1    4350 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5E4A7F97
P 4350 5350
F 0 "C2" H 4200 5250 50  0000 L CNN
F 1 "4.7uF" H 4050 5350 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 4388 5200 50  0001 C CNN
F 3 "~" H 4350 5350 50  0001 C CNN
	1    4350 5350
	1    0    0    -1  
$EndComp
$Comp
L StupidDogFocuserPCB-rescue:SN74HC14N-dk_Logic-Gates-and-Inverters-StupidDogFocuserPCB-rescue U2
U 1 1 5E4A594F
P 2600 5500
F 0 "U2" H 2600 4797 60  0000 C CNN
F 1 "SN74HC14N" H 2600 4903 60  0000 C CNN
F 2 "digikey-footprints:DIP-14_W3mm" H 2800 5700 60  0001 L CNN
F 3 "http://www.ti.com/general/docs/suppproductinfo.tsp?distId=10&gotoUrl=http%3A%2F%2Fwww.ti.com%2Flit%2Fgpn%2Fsn74hc14" H 2800 5800 60  0001 L CNN
F 4 "296-1577-5-ND" H 2800 5900 60  0001 L CNN "Digi-Key_PN"
F 5 "SN74HC14N" H 2800 6000 60  0001 L CNN "MPN"
F 6 "Integrated Circuits (ICs)" H 2800 6100 60  0001 L CNN "Category"
F 7 "Logic - Gates and Inverters" H 2800 6200 60  0001 L CNN "Family"
F 8 "http://www.ti.com/general/docs/suppproductinfo.tsp?distId=10&gotoUrl=http%3A%2F%2Fwww.ti.com%2Flit%2Fgpn%2Fsn74hc14" H 2800 6300 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/texas-instruments/SN74HC14N/296-1577-5-ND/277223" H 2800 6400 60  0001 L CNN "DK_Detail_Page"
F 10 "IC INVERTER SCHMITT 6CH 14DIP" H 2800 6500 60  0001 L CNN "Description"
F 11 "Texas Instruments" H 2800 6600 60  0001 L CNN "Manufacturer"
F 12 "Active" H 2800 6700 60  0001 L CNN "Status"
	1    2600 5500
	-1   0    0    1   
$EndComp
$Comp
L Sensor:DHT11 U1
U 1 1 5E4A25BA
P 7650 5250
F 0 "U1" V 7250 5500 50  0000 R CNN
F 1 "DHT11" V 7350 5600 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 7650 4850 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 7800 5500 50  0001 C CNN
	1    7650 5250
	0    1    1    0   
$EndComp
$Comp
L Driver_Motor:Pololu_Breakout_DRV8825 A1
U 1 1 5E4A1255
P 6300 2300
F 0 "A1" H 6700 1850 50  0000 C CNN
F 1 "Pololu_Breakout_DRV8825" H 7150 1750 50  0000 C CNN
F 2 "Module:Pololu_Breakout-16_15.2x20.3mm" H 6500 1500 50  0001 L CNN
F 3 "https://www.pololu.com/product/2982" H 6400 2000 50  0001 C CNN
	1    6300 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:Rotary_Encoder_Switch SW1
U 1 1 5E4A009D
P 5100 5550
F 0 "SW1" H 5100 5800 50  0000 C CNN
F 1 "Rotary_Encoder_Switch" H 5200 5250 50  0000 C CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC11E-Switch_Vertical_H20mm" H 4950 5710 50  0001 C CNN
F 3 "~" H 5100 5810 50  0001 C CNN
	1    5100 5550
	1    0    0    -1  
$EndComp
$Comp
L MCU_Module:Arduino_UNO_R3 A2
U 1 1 5E49DA47
P 2750 2350
F 0 "A2" H 2050 3400 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 2050 3300 50  0000 C CNN
F 2 "Module:Arduino_UNO_R3" H 2750 2350 50  0001 C CIN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 2750 2350 50  0001 C CNN
	1    2750 2350
	1    0    0    -1  
$EndComp
NoConn ~ 3250 1750
NoConn ~ 3250 1950
NoConn ~ 3250 2150
NoConn ~ 3250 2350
NoConn ~ 3250 2450
NoConn ~ 3250 3050
NoConn ~ 3250 3150
NoConn ~ 2250 2750
NoConn ~ 2250 2650
NoConn ~ 2250 2550
NoConn ~ 2250 2150
NoConn ~ 2250 1850
NoConn ~ 2250 1750
NoConn ~ 5900 1900
Wire Wire Line
	2150 2950 2250 2950
NoConn ~ 2250 3050
Text GLabel 5800 2800 0    50   Input ~ 0
M1
Text GLabel 5800 2900 0    50   Input ~ 0
M2
Text GLabel 3350 2750 2    50   Input ~ 0
M1
Text GLabel 3350 2850 2    50   Input ~ 0
M2
Wire Wire Line
	3350 2550 3250 2550
Wire Wire Line
	3250 2650 3350 2650
Wire Wire Line
	5800 2800 5900 2800
Wire Wire Line
	5900 2900 5800 2900
Wire Wire Line
	4800 5100 4800 5450
Connection ~ 4800 5450
Wire Wire Line
	4800 6000 4800 5650
Connection ~ 4800 5650
Wire Wire Line
	5400 5100 5400 5450
Connection ~ 5400 5450
Text GLabel 2150 2850 0    50   Input ~ 0
LIM
Wire Wire Line
	2150 2850 2250 2850
Text GLabel 2150 5500 0    50   Input ~ 0
LIM
Wire Wire Line
	2150 5500 2300 5500
Text GLabel 2950 5500 2    50   Input ~ 0
LIM_A
Wire Wire Line
	2900 5500 2950 5500
$Comp
L StupidDogFocuserPCB-rescue:OSTTC062162-dk_Terminal-Blocks-Wire-to-Board-StupidDogFocuserPCB-rescue J1
U 1 1 5E624889
P 8350 2250
F 0 "J1" V 8119 2233 60  0000 C CNN
F 1 "OSTTC062162" V 8217 2233 50  0000 C CNN
F 2 "digikey-footprints:Term_Block_1x6_P5.08mm" H 8550 2450 60  0001 L CNN
F 3 "http://www.on-shore.com/wp-content/uploads/OSTTCXX2162.pdf" H 8550 2550 60  0001 L CNN
F 4 "ED2611-ND" H 8550 2650 60  0001 L CNN "Digi-Key_PN"
F 5 "OSTTC042162" H 8550 2750 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 8550 2850 60  0001 L CNN "Category"
F 7 "Terminal Blocks - Wire to Board" H 8550 2950 60  0001 L CNN "Family"
F 8 "http://www.on-shore.com/wp-content/uploads/OSTTCXX2162.pdf" H 8550 3050 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-shore-technology-inc/OSTTC042162/ED2611-ND/614560" H 8550 3150 60  0001 L CNN "DK_Detail_Page"
F 10 "TERM BLK 4P SIDE ENT 5.08MM PCB" H 8550 3250 60  0001 L CNN "Description"
F 11 "On Shore Technology Inc." H 8550 3350 60  0001 L CNN "Manufacturer"
F 12 "Active" H 8550 3450 60  0001 L CNN "Status"
	1    8350 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	8450 2250 8650 2250
Wire Wire Line
	8450 2350 8650 2350
Wire Wire Line
	8450 2450 8650 2450
Wire Wire Line
	8450 2550 8650 2550
Text GLabel 8650 2650 2    50   Input ~ 0
LIM_SW
Text GLabel 8650 2750 2    50   Input ~ 0
GND
Wire Wire Line
	8450 2650 8650 2650
Wire Wire Line
	8450 2750 8650 2750
Wire Wire Line
	8050 4200 8200 4200
Wire Wire Line
	8450 3900 8550 3900
Connection ~ 8450 3900
Wire Wire Line
	8450 3950 8450 3900
Wire Wire Line
	8200 3900 8450 3900
Text GLabel 8550 3900 2    50   Input ~ 0
LIM_A
Wire Wire Line
	8050 3550 7850 3550
$Comp
L Device:R R7
U 1 1 5E63E909
P 8050 3700
F 0 "R7" H 8120 3746 50  0000 L CNN
F 1 "1k" H 8120 3655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 7980 3700 50  0001 C CNN
F 3 "~" H 8050 3700 50  0001 C CNN
	1    8050 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5E63E913
P 8200 4050
F 0 "R8" H 8270 4096 50  0000 L CNN
F 1 "1k" H 8270 4005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8130 4050 50  0001 C CNN
F 3 "~" H 8200 4050 50  0001 C CNN
	1    8200 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5E63E91D
P 8450 4100
F 0 "C6" H 8565 4146 50  0000 L CNN
F 1 "4.7uF" H 8565 4055 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 8488 3950 50  0001 C CNN
F 3 "~" H 8450 4100 50  0001 C CNN
	1    8450 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 3850 8050 4200
Text GLabel 7900 4200 0    50   Input ~ 0
LIM_SW
Wire Wire Line
	7900 4200 8050 4200
Connection ~ 8050 4200
Text GLabel 7850 3550 0    50   Input ~ 0
VCC
Text GLabel 8450 4250 3    50   Input ~ 0
GND
$EndSCHEMATC
