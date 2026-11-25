Connectors and Switches
#######################

The LimeRFE 8001P board features various connectors and test switches. The top and bottom views of the board, with the locations of these components highlighted, are shown in Figures 6 and 7, respectively.

.. figure:: /images/LimeRFE-8001P_v1.1_con_top.png
  :width: 600

  Figure 6: LimeRFE 8001P v1.1 connectors and switches - top view

.. figure:: /images/LimeRFE-8001P_v1.1_con_bot.png
  :width: 600

  Figure 7: LimeRFE 8001P v1.1 connectors - bottom view

.. list-table:: Table 2. Connectors and switches
   :header-rows: 1

   * - Connector
     - Type
     - Description

   * - J1
     - I2C connector
     - I2C connector for front panel communication. Allowed voltage range for this connector is 2.3 – 5.5 V.
       
       This is a 5 pin connector:
       
       PIN1: GND
       
       PIN2: VDD_CON_J1
       
       PIN3: RPI_ID_SC_EXT
       
       PIN4: RPI_ID_SD_EXT
       
       PIN5: RPI_BTN2_EXT

   * - J2
     - DC input
     - 5 V power supply input for the LimeRFE 8001P board supply.
       
       For further details about DC bias selection, please see :doc:`power`.

   * - J3
     - RF input
     - External LO signal for LMS8001B on channel 1.
       
       In the default configuration the signal from the J3 connector is brought at both LMS8001B chips, 
       
       due to mutual synchronization.
       
       External LO signal for LMS8001B on channel 2 is J26 connector.

   * - J4
     - RF input
     - ORX1 input
      
       Signal from this port is taken from an external PA output (by directional coupler) and it goes to the RFFB input
      
       of a SC1905 chip on channel 1, in order to linearize that external PA.

   * - J5
     - RF input
     - ORX2 input
       
       Signal from this port is taken from an external PA output (by directional coupler) and it goes to the RFFB input
       
       of a SC1905 chip on channel 2, in order to linearize that external PA.

   * - J6
     - IF input
     - IFTX1

   * - J7
     - IF output
     - IFORX1
       
       Signal from connector J4 (after a frequency shifting obtained in LMS8001B chip on channel 1) can be brought to this 
       
       connector J7. To achieve this R260 must be not fitted, and R261 fitted. Additionally, R191 must be not fitted and 
       
       R193 must be fitted.

   * - J8
     - IF output
     - IFRX1

   * - J9
     - IF input
     - IFTX2

   * - J10
     - IF output
     - IFORX2
       
       Signal from connector J5 (after a frequency shifting obtained in LMS8001B chip on channel 2) can be brought to this 
       
       connector J10. To achieve this R266 must be not fitted, and R267 fitted. Additionally, R192 must be not fitted and 
       
       R194 must be fitted.

   * - J11
     - IF output
     - IFRX2

   * - J12
     - Jumper shunt
     - Jumper shunt (Jumper_SNM-100-BK-T) in the board power supply selection circuit.

       It is a BOM item, not a connector.

   * - J13
     - REF input
     - 10 MHz external reference signal for external PLL tune.

   * - J14
     - REF out
       (square signal)
     - Output from the clock buffer U21 (NB3H83905CMNG) in the external PLL circuit, for clock monitoring.

   * - J15
     - RF input/output
     - TXRX1
       
       RF output for TXRX_1 = 1.
       
       RF input for TXRX_1 = 0.

   * - J16
     - DC input
     - Additional 5 V DC supply (+5V_RF) for RF devices:
       
       AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1 and LNA2.

   * - J17
     - RF input
     - RX1
       
       Dedicated RF input for LNA1. To use this connector R565 must be fitted, and C233 must be not fitted.

   * - J18
     - Digital input
     - Dedicated TDD control signal for DA1_A. Signal levels at logic state “1” must be in 3 – 5 V range.
       
       When signal is at logic “1” DA1_A (U58) is turned on.
       
       When signal is at logic “0” DA1_A (U58) is turned off.

   * - J19
     - SPI2 connector
     - SPI2 bus is connection between MCU and devices on the board. MCU on the LimeRFE 8001P board is a master on 
       
       SPI2 bus, connector J19. Allowed voltage for this connector is 3.3 V.
       
       This (SPI2) is a 6 pin connector:
       
       PIN1: MISO_2 – Master input, slave output on SPI2 bus.
       
       PIN2: VDIO – 3.3 V DC supply, obtained from the board.
       
       PIN3: SCLK_2 – Clock signal for SPI2 bus
       
       PIN4: MOSI_2 – Master output, slave input on SPI2 bus.
       
       PIN5: RESET – MCU reset.
       
       PIN6: GND

   * - J20
     - RF input/output
     - TXRX2
       
       RF output for TXRX_2 = 1.
       
       RF input for TXRX_2 = 0.

   * - J21
     - DC input
     - Additional 5 V DC supply (+5V_RF) for RF devices:
       
       AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1 and LNA2.

   * - J22
     - RF input
     - RX2
       
       Dedicated RF input for LNA2. To use this connector R572 must be fitted, and C328 must be not fitted.

   * - J23
     - Digital signal input
     - Dedicated TDD control signal for DA2_A. Signal levels at logic state “1” must be in 3–5 V range.
       
       When signal is at logic “1” DA2_A (U69) is turned on.
       
       When signal is at logic “0” DA2_A (U69) is turned off.

   * - J24
     - DC input
     - 5 V supply for RF devices:
       
       AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1 and LNA2.
       
       For further details about DC bias selection, please see :doc:`power`.

   * - J25
     - Jumper shunt
     - Jumper shunt (Jumper_SNM-100-BK-T) in the RF devices power supply selection circuit.

       It is a BOM item, not a connector.

   * - J26
     - RF input
     - External LO signal for LMS8001B on channel 2.
       
       In the default configuration the signal from the J26 connector is brought at both LMS8001B chips, 
       
       due to mutual synchronization.
       
       External LO signal for LMS8001B on channel 1 is J3 connector.

   * - J27
     - DC input
     - 12 V power supply input (3 pin header).

       It supplies the on-board DC-DC converter, which generates 5 V for the board supply.

       The other 12 V inputs are J30 and J45.
       
       For further details about DC bias selection, please see :doc:`power`.

   * - J28
     - Digital signals
  
       (SPI1 bus) connector
     - SPI1 bus is connection between MCU and the external control board. MCU on the LimeRFE 8001P board is a slave on 
      
       SPI1 bus, connector J28. Allowed voltage range for this connector is 1.65 – 5.5 V.
       
       This (SPI1) is 12 pin connector:
       
       PIN1: SS_1_EXT – slave selection on SPI bus
       
       PIN2: MOSI_1_EXT – Master output, slave input on SPI1 bus
       
       PIN3: MISO_1_EXT – Master input, slave output on SPI1 bus
       
       PIN4: SCLK_1_EXT – Clock signal on SPI1 bus
       
       PIN5: GND
       
       PIN6: External power supply which should correspond to SPI1 bus signal levels.
       
       PIN7: TX/rx_1_EXT – Interrupt signal for MCU, for controlling TDD mode operation on channel 1.
       
       PIN8: TX/rx_2_EXT – Interrupt signal for MCU, for controlling TDD mode operation on channel 2.
       
       PIN9: MCU_GPIO18_EXT, GPIO18 pin brought from MCU (not connected by default, R177 not fitted).
       
       PIN10: MCU_GPIO19_EXT, GPIO19 pin brought from MCU (not connected by default, R178 not fitted).
       
       PIN11: GND
       
       PIN12: External power supply which should correspond to SPI1 bus signal levels.

   * - J29
     - USB connector
     - USB connector, 2x5 pin header with pin 9 removed for keying. Pins brought to the header are:
       
       PIN1: VBUS_USB
       
       PIN2: VBUS_USB
       
       PIN3: USB_DM1
       
       PIN4: USB_DM1
       
       PIN5: USB_DP1
       
       PIN6: USB_DP1
       
       PIN7: GND
       
       PIN8: GND
       
       PIN10: Not connected.

   * - J30
     - DC input
     - 12 V power supply input (DC barrel jack).

       It supplies the on-board DC-DC converter, which generates 5 V for the board supply.

       The other 12 V inputs are J27 and J45.
       
       For further details about DC bias selection, please see :doc:`power`.

   * - J31
     - Does not exist
     -

   * - J32
     - Does not exist
     -

   * - J33
     - IF input
     - This is U.FL connector on the board channel 1 (CH#1), and LMS8001B’s channel A, (Ch.A). Default configuration for 
      
       (CH#1, Ch.A) is up-conversion, where J33 is input and accordingly it is in IF frequency range. Output from 
       
       (CH#1, Ch.A) is J34 connector, and signals at this connector are in RF range.
       
       In the default state (CH#1, Ch.A) this is not used and J33 should be terminated with 50 Ohm resistor R68, 
       
       and J34 connector should be terminated with 50 Ohm resistor R570.

   * - J34
     - RF output
     - This is U.FL connector on the board channel 1 (CH#1), and LMS8001B channel A, (Ch.A). Default configuration for 
      
       (CH#1, Ch.A) is up-conversion, where J33 is input and accordingly it is in IF frequency range. Output from 
       
       (CH#1, Ch.A) is J34 connector, and signals at this connector are in RF range.
       
       In the default state (CH#1, Ch.A) this is not used and J33 should be terminated with 50 Ohm resistor R68, 
       
       and J34 connector should be terminated with 50 Ohm resistor R570.

   * - J35
     - RF input
     - This is U.FL connector on LMS8001B on the board channel 2 (CH#2), and LMS8001B channel A, (Ch.A). 
     
       Default configuration for (CH#2, Ch.A) is down-conversion, where J35 is input and accordingly it is in RF 
       
       frequency range. Output from (CH#2, Ch.A) is J57 connector, and signals at this connector are in IF range.
       
       In the default state (CH#2, Ch.A) this is not used and J35 should be terminated with 50 Ohm resistor R574, 
       
       and J57 connector should be terminated with 50 Ohm resistor R575.

   * - J36
     - PMOD connector for free
  
       MCU GPIO signals
     - This connector provides access to free GPIO signals of the board MCU. Allowed voltage range for this connector
   
       is 1.65 – 5.5 V.
       
       This is 12 pin connector:
       
       PIN1: MCU_GPIO13_EXT
       
       PIN2: MCU_GPIO23_EXT
       
       PIN3: MCU_GPIO24_EXT
       
       PIN4: MCU_GPIO25_EXT
       
       PIN5: GND
       
       PIN6: External DC supply, 3.3 – 5 V
       
       PIN7: MCU_GPIO26_EXT
       
       PIN8: MCU_GPIO27_EXT
       
       PIN9: MCU_GPIO28_EXT
       
       PIN10: MCU_GPIO29_EXT
       
       PIN11: GND
       
       PIN12: External DC supply, 3.3 – 5 V

   * - J37
     - Digital signal input
     - Dedicated TDD control signal for DA1_B. Signal levels at logic state “1” should be 1.8 V.
       
       When signal is at logic “1” DA1_B (U77) is turned on.
       
       When signal is at logic “0” DA1_B (U77) is turned off.

   * - J38
     - Does not exist
     -

   * - J39
     - Does not exist
     -

   * - J40
     - Digital signal input
     - Dedicated TDD control signal for DA2_B. Signal levels at logic state “1” should be 1.8 V.
       
       When signal is at logic “1” DA2_B (U79) is turned on.
       
       When signal is at logic “0” DA2_B (U79) is turned off.

   * - J41
     - Does not exist
     -

   * - J42
     - RF input
     - This is RF signal input for power detector unit which is not fitted on the LimeRFE 8001P board. There is an option to 
      
       use component IC1 (ADL5513ACPZ-R7) and its output signal to return to SC1905 on channel 1. This power detector 
       
       unit option is in original SC1905 hardware design document.

   * - J43
     - Clock input
     - Optional external clock input (EXTCLKIN) for the SC1905 on channel 1.

       Not connected by default (R403 not fitted); the SC1905 uses the on-board 20 MHz crystal X1.

   * - J44
     - Does not exist
     -

   * - J45
     - DC input
     - 12 V power supply input (2x3 pin power connector).

       It supplies the on-board DC-DC converter, which generates 5 V for the board supply.

       The other 12 V inputs are J27 and J30.
       
       For further details about DC bias selection, please see :doc:`power`.

   * - J46
     - MCU bootsel 
      
       mode selection
     - This connector is used in the programming the MCU. When J46 is shorted the MCU enters in “bootsel”, after which 
      
       it functions as external storage and code can be copied directly to it.

   * - J47
     - Jumper shunt
     - Jumper shunt (Jumper_SNM-100-BK-T) in the MCU circuit.

       It is a BOM item, not a connector.

   * - J48
     - Does not exist
     -

   * - J49
     - RF input
     - This is RF signal input for power detector unit which is not fitted on the LimeRFE 8001P board. There is an option to 
      
       use component IC2 (ADL5513ACPZ-R7) and its output signal to return to SC1905 on channel 2. This power detector 
       
       unit option is in original SC1905 hardware design document.

   * - J50
     - Clock input
     - Optional external clock input (EXTCLKIN) for the SC1905 on channel 2.

       Not connected by default (R502 not fitted); the SC1905 uses the on-board 20 MHz crystal X2.

   * - J51
     - Does not exist
     -

   * - J52
     - Does not exist
     -

   * - J53
     - Does not exist
     -

   * - J54
     - Does not exist
     -

   * - J55
     - Does not exist
     -

   * - J56
     - Does not exist
     -

   * - J57
     - IF output
     - This is U.FL connector on the board channel 2 (CH#2), and LMS8001B’s channel A, (Ch.A). Default configuration for 
      
       (CH#2, Ch.A) is down-conversion, where J35 is input and accordingly it is in RF frequency range. Output from 
       
       (CH#2, Ch.A) is J57 connector, and signals at this connector are in IF range.
       
       In default state (CH#2, Ch.A) is not used and J35 should be terminated with 50 Ohm resistor R574, 
       
       and J57 connector should be terminated with 50 Ohm resistor R575.

   * - JP1
     - The board DC 
      
       supply selection
     - This connector selects main board supply. There are 3 possible options:
       
       External 12 V supply (Pins 1-2, jumper top pair)
       
       USB supply (Pins 3-4, jumper center pair)
       
       External 5 V supply (Pins 5-6, jumper bottom pair)
       
       For detail description and recommended options, related to this connector, please refer to :doc:`power`.

   * - JP2
     - RF devices 
      
       supply selection
     - This connector selects RF devices supply. There are 3 possible options:
       
       The board supply (Pins 1-2, jumper top pair)
       
       USB supply (Pins 3-4, jumper center pair)
       
       External 5 V supply (Pins 5-6, jumper bottom pair)
       
       For detail description and recommended options, related to this connector, please refer to :doc:`power`.

   * - SW2
     - Reset taster
     - Reset for micro-controller (MCU).

   * - SW3
     - Reset taster
     - Reset for SC1905 on channel 1.

   * - SW4
     - Reset taster
     - Reset for SC1905 on channel 2.

   * - CON1
     - USB connector
     - USB connector, type B micro.
