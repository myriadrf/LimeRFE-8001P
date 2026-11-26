Connectors and Switches
#######################

The LimeRFE 8001P v1.0 board features various connectors and test switches. The view of the board, with the locations of these components highlighted, is shown in Figure 6.

.. figure:: /images/LimeRFE-8001P_v1.0_con_top.png
  :width: 600

  Figure 6: LimeRFE 8001P v1.0 connectors and switches - top view

.. list-table:: Table 2. Connectors and switches
   :header-rows: 1

   * - Connector
     - Type
     - Description
   * - J1
     - REF input
     - Alternative 10 MHz external reference input, connected in parallel with J13.

       Not fitted in the default configuration.
   * - J2
     - DC input
     - 5 V power supply input for the LimeRFE 8001P board supply.

       For further details about DC bias selection, please see :doc:`power`.
   * - J3
     - RF input
     - External LO signal for LMS8001B on channel 1.

       In the default configuration the signal from the J3 connector is brought to both LMS8001B chips,

       due to mutual synchronization.
   * - J4
     - RF input
     - ORX1 input

       Signal from this port is taken from an external PA output (by directional coupler).

       It goes to the RFFB input of the SC1905 chip on channel 1, in order to linearize that external PA.
   * - J5
     - RF input
     - ORX2 input

       Signal from this port is taken from an external PA output (by directional coupler).

       It goes to the RFFB input of the SC1905 chip on channel 2, in order to linearize that external PA.
   * - J6
     - IF input
     - IFTX1
   * - J7
     - IF output
     - IFORX1

       The J4 signal, after frequency shifting in the LMS8001B on channel 1, can be brought to this connector.

       See :ref:`orx-reconfiguration-v1.0`.
   * - J8
     - IF output
     - IFRX1
   * - J9
     - IF input
     - IFTX2
   * - J10
     - IF output
     - IFORX2

       The J5 signal, after frequency shifting in the LMS8001B on channel 2, can be brought to this connector.

       See :ref:`orx-reconfiguration-v1.0`.
   * - J11
     - IF output
     - IFRX2
   * - J12
     - Jumper shunt
     - Jumper shunt (Jumper_SNM-100-BK-T). It is a BOM item, not a connector.
   * - J13
     - REF input
     - 10 MHz external reference signal for external PLL tune.
   * - J14
     - REF out (square signal)
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

       For test and debug purposes only.
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
     - SPI2 bus connector (MCU is the master), see :doc:`peripheralinterfaces`.
   * - J20
     - RF input/output
     - TXRX2

       RF output for TXRX_2 = 1.

       RF input for TXRX_2 = 0.
   * - J21
     - DC input
     - Additional 5 V DC supply (+5V_RF) for RF devices:

       AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1 and LNA2.

       For test and debug purposes only.
   * - J22
     - RF input
     - RX2

       Dedicated RF input for LNA2. To use this connector R572 must be fitted, and C328 must be not fitted.
   * - J23
     - Digital input
     - Dedicated TDD control signal for DA2_A. Signal levels at logic state “1” must be in 3 – 5 V range.

       When signal is at logic “1” DA2_A (U69) is turned on.

       When signal is at logic “0” DA2_A (U69) is turned off.
   * - J24
     - DC input
     - 5 V supply for RF devices:

       AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1 and LNA2.

       For further details about DC bias selection, please see :doc:`power`.
   * - J25
     - Jumper shunt
     - Jumper shunt (Jumper_SNM-100-BK-T). It is a BOM item, not a connector.
   * - J26
     - RF input
     - External LO signal for LMS8001B on channel 2.

       Not used in the default configuration, because the synchronization TCXO PLL on channel 2 is not fitted.
   * - J27
     - DC input
     - 12 V power supply input (3 pin header) for the DC-DC converter.

       The 5 V output of the DC-DC converter can be used for the board supply.

       For further details about DC bias selection, please see :doc:`power`.
   * - J28
     - SPI1 connector
     - SPI1 bus connector for the external control board (MCU is the slave), see :doc:`peripheralinterfaces`.
   * - J29
     - REF out (square signal)
     - Alternative clock monitoring output. Not fitted in the default configuration.
   * - J30
     - PMOD connector
     - GPIO signals of the LMS8001B on channel 1, see :doc:`gpios`.
   * - J31
     - PMOD connector
     - GPIO signals of the LMS8001B on channel 2, see :doc:`gpios`.
   * - J32
     - SC1905 debug connector
     - 14-pin connector for a Maxim controller board, for SC1905 on channel 1, see :doc:`peripheralinterfaces`.
   * - J33
     - IF input
     - U.FL connector, LMS8001B *Ch.B* input on channel 1 (up-conversion).

       Not used. J33 and its 50 Ohm terminating resistor R68 are not fitted.
   * - J34
     - RF output
     - U.FL connector, LMS8001B *Ch.B* output on channel 1 (up-conversion).

       Not used. J34 and its 50 Ohm terminating resistor R570 are not fitted.
   * - J35
     - RF input
     - U.FL connector, LMS8001B *Ch.B* input on channel 2 (down-conversion).

       Not used; terminated with 50 Ohm resistor R574.
   * - J36
     - PMOD connector
     - Free MCU GPIO signals, see :doc:`gpios`.
   * - J37
     - Digital input
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
     - Digital input
     - Dedicated TDD control signal for DA2_B. Signal levels at logic state “1” should be 1.8 V.

       When signal is at logic “1” DA2_B (U79) is turned on.

       When signal is at logic “0” DA2_B (U79) is turned off.
   * - J41
     - Does not exist
     -
   * - J42
     - RF input
     - RF input of the power detector sub-block on channel 1. The power detector IC1 (ADL5513ACPZ-R7) is not fitted.

       This option comes from the original SC1905 hardware design.
   * - J43
     - Clock
     - Access to the clock signal of the SC1905 on channel 1. Not used in the default configuration.
   * - J44
     - DC input
     - 12 V power supply input (2 pin header) for the DC-DC converter.

       The 5 V output of the DC-DC converter can be used for the board supply.

       For further details about DC bias selection, please see :doc:`power`.
   * - J45
     - DC input
     - 12 V power supply input (2x3 pin power connector) for the DC-DC converter.

       For further details about DC bias selection, please see :doc:`power`.
   * - J46
     - MCU bootsel mode selection
     - This connector is used for programming the MCU. When J46 is shorted the MCU enters the “bootsel” mode.

       In this mode it functions as external storage and code can be copied directly to it.
   * - J47
     - Jumper shunt
     - Jumper shunt (Jumper_SNM-100-BK-T). It is a BOM item, not a connector.
   * - J48
     - SC1905 debug connector
     - 14-pin connector for a Maxim controller board, for SC1905 on channel 2, see :doc:`peripheralinterfaces`.
   * - J49
     - RF input
     - RF input of the power detector sub-block on channel 2. The power detector IC2 (ADL5513ACPZ-R7) is not fitted.

       This option comes from the original SC1905 hardware design.
   * - J50
     - Clock
     - Access to the clock signal of the SC1905 on channel 2. Not used in the default configuration.
   * - J51
     - PMOD connector
     - Digital signals of the SC1905 on channel 2, see :doc:`peripheralinterfaces`.
   * - J52
     - PMOD connector
     - Digital signals of the SC1905 on channel 1, see :doc:`peripheralinterfaces`.
   * - J53
     - 2 pin header
     - When a jumper is placed on J53, the SC1905 on channel 1 is connected to the board MCU through the SPI2 bus.

       Remove the jumper when J32 is used.
   * - J54
     - Jumper shunt
     - Jumper shunt (Jumper_SNM-100-BK-T). It is a BOM item, not a connector.
   * - J55
     - 2 pin header
     - When a jumper is placed on J55, the SC1905 on channel 2 is connected to the board MCU through the SPI2 bus.

       Remove the jumper when J48 is used.
   * - J56
     - Jumper shunt
     - Jumper shunt (Jumper_SNM-100-BK-T). It is a BOM item, not a connector.
   * - J57
     - IF output
     - U.FL connector, LMS8001B *Ch.B* output on channel 2 (down-conversion).

       Not used; terminated with 50 Ohm resistor R575.
   * - JP1
     - Board DC supply selection
     - Selects the main board supply:

       External 12 V supply (Pins 1-2, jumper top pair)

       USB supply (Pins 3-4, jumper center pair)

       External 5 V supply (Pins 5-6, jumper bottom pair)

       For recommended options, please refer to :doc:`power`.
   * - JP2
     - RF devices supply selection
     - Selects the RF devices supply:

       The board supply (Pins 1-2, jumper top pair)

       USB supply (Pins 3-4, jumper center pair)

       External 5 V supply (Pins 5-6, jumper bottom pair)

       For recommended options, please refer to :doc:`power`.
   * - SW2
     - Reset switch
     - Reset for micro-controller (MCU).
   * - SW3
     - Reset switch
     - Reset for SC1905 on channel 1.
   * - SW4
     - Reset switch
     - Reset for SC1905 on channel 2.
   * - CON1
     - USB connector
     - USB connector, type B micro.

