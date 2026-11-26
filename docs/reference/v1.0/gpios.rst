GPIOs and Control Signals
#########################

GPIO Connector
==============

The free GPIO pins of the board MCU (U115, RP2040) are available on the 12-pin PMOD connector J36. The signals pass through the level shifter U38.
J36 pinout is given in Table 8.

.. list-table:: Table 8. GPIO PMOD connector (J36) pinout
   :header-rows: 1

   * - Pin
     - Signal
     - Description
   * - 1
     - MCU_GPIO13_EXT
     - MCU GPIO13
   * - 2
     - MCU_GPIO23_EXT
     - MCU GPIO23
   * - 3
     - MCU_GPIO24_EXT
     - MCU GPIO24
   * - 4
     - MCU_GPIO25_EXT
     - MCU GPIO25
   * - 5
     - GND
     - Ground
   * - 6
     - Supply
     - DC supply input, must be in the range 3.3 – 5 V
   * - 7
     - MCU_GPIO26_EXT
     - MCU GPIO26
   * - 8
     - MCU_GPIO27_EXT
     - MCU GPIO27
   * - 9
     - MCU_GPIO28_EXT
     - MCU GPIO28
   * - 10
     - MCU_GPIO29_EXT
     - MCU GPIO29
   * - 11
     - GND
     - Ground
   * - 12
     - Supply
     - DC supply input, must be in the range 3.3 – 5 V

TDD Control
===========

The driver amplifiers can be switched on and off by external signals, for TDD operation. Each driver amplifier branch has a dedicated 2-pin header, listed in Table 9.

.. list-table:: Table 9. External TDD control headers
   :header-rows: 1

   * - Header
     - Controlled device
     - Logic "1" level
   * - J18
     - DA1_A (U58), channel 1, branch A
     - 3 – 5 V
   * - J37
     - DA1_B (U77), channel 1, branch B
     - 1.8 V
   * - J23
     - DA2_A (U69), channel 2, branch A
     - 3 – 5 V
   * - J40
     - DA2_B (U79), channel 2, branch B
     - 1.8 V

When the signal is at logic "1", the driver amplifier is turned on. When the signal is at logic "0", the driver amplifier is turned off.

TDD operation can also be controlled through the MCU, using the TX/rx_1_EXT and TX/rx_2_EXT interrupt signals on pins 7 and 8 of the SPI1 connector J28 (see :doc:`peripheralinterfaces`), or from the GUI.

.. Note: the source manual describes J37 as controlling DA2_B (U79). The v1.0 netlist confirms that J37 drives
   DA1_B (U77) through the U76 buffer and R146.

LMS8001B GPIO Connectors
========================

The GPIO signals of the LMS8001B chips are available on two 12-pin PMOD connectors: J30 for channel 1 and J31 for channel 2. The signals pass through the level shifters U25 and U70 (J30), and U71 and U72 (J31).
J30 and J31 pinout is given in Table 10, where *n* is 1 for J30 and 2 for J31.

.. list-table:: Table 10. LMS8001B GPIO connectors (J30, J31) pinout
   :header-rows: 1

   * - Pin
     - Signal
     - Description
   * - 1
     - GPIO0_#n_OUT
     - LMS8001B GPIO0
   * - 2
     - GPIO1_#n_OUT
     - LMS8001B GPIO1
   * - 3
     - GPIO2_#n_OUT
     - LMS8001B GPIO2
   * - 4
     - GPIO3_#n_OUT
     - LMS8001B GPIO3
   * - 5
     - GND
     - Ground
   * - 6
     - Supply
     - Default: DC supply input, must be in the range 3.3 – 5 V (R302 / R303 fitted, R300 / R301 not fitted).

       Option: VDIO (3.3 V) from the board (R302 / R303 not fitted, R300 / R301 fitted).
   * - 7
     - GPIO4_#n_OUT
     - LMS8001B GPIO4
   * - 8
     - GPIO5_#n_OUT
     - LMS8001B GPIO5
   * - 9
     - GPIO6_#n_OUT
     - LMS8001B GPIO6
   * - 10
     - GPIO7_#n_OUT
     - LMS8001B GPIO7
   * - 11
     - GND / GPIO8_#n_OUT
     - Default: Ground (R306 / R307 fitted, R304 / R305 not fitted).

       Option: LMS8001B GPIO8 (R306 / R307 not fitted, R304 / R305 fitted).
   * - 12
     - Supply
     - Default: DC supply input, must be in the range 3.3 – 5 V (R318 / R319 fitted, R320 / R321 not fitted).

       Option: VDIO (3.3 V) from the board (R318 / R319 not fitted, R320 / R321 fitted).

In the resistor lists above, the first designator applies to J30 and the second to J31.

.. Note: the source manual gives R318 / R320 for pin 12 of both J30 and J31. R319 / R321 for J31 are taken
   from the v1.0 netlist.
