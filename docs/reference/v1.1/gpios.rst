GPIOs and Control Signals
#########################

GPIO Connector
**************

The free GPIO pins of the board MCU (U115, RP2040) are available on the 12-pin PMOD connector J36. The signals pass through the level shifter U38.
The allowed voltage range for this connector is 1.65 – 5.5 V. The voltage applied to pins 6 and 12 (VDD_CON_J36) sets the signal levels on the connector side of the level shifter.
The level shifter direction is controlled by MCU GPIO20. J36 pinout is given in Table 8.

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
     - VDD_CON_J36
     - Reference voltage input, see the allowed range above
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
     - VDD_CON_J36
     - Reference voltage input, see the allowed range above

Two more MCU GPIOs (GPIO18 and GPIO19) can be routed to pins 9 and 10 of the SPI1 connector J28 by fitting R177 and R178, see :doc:`peripheralinterfaces`.

.. TODO: the source manual gives 3.3 – 5 V for the J36 supply pins. Confirm the range with the designers.

TDD Control
***********

The driver amplifiers can be switched on and off by external signals, for TDD operation. Each driver amplifier branch has a dedicated 2-pin header, listed in Table 9.

.. list-table:: Table 9. External TDD control headers
   :header-rows: 1

   * - Header
     - Controlled device
     - Pinout
     - Logic "1" level
   * - J18
     - DA1_A (U58), channel 1, branch A
     - Pin 1 GND, pin 2 signal
     - 3 – 5 V
   * - J37
     - DA1_B (U77), channel 1, branch B
     - Pin 1 signal, pin 2 GND
     - 1.8 V
   * - J23
     - DA2_A (U69), channel 2, branch A
     - Pin 1 GND, pin 2 signal
     - 3 – 5 V
   * - J40
     - DA2_B (U79), channel 2, branch B
     - Pin 1 signal, pin 2 GND
     - 1.8 V

When the signal is at logic "1", the driver amplifier is turned on. When the signal is at logic "0", the driver amplifier is turned off.

J18 and J23 are connected to the enable net at the input of the on-board control buffer (U59, U74).
J37 and J40 are connected to the output of the on-board control buffer (U76, U78), which drives the QPA9122M power-down pin.

.. warning::
   The pin order of J37 and J40 is reversed compared to J18 and J23.

.. TODO: the TDD headers are connected to nets that are also driven by the on-board control logic
   (DA enable gates and buffers). Ask the designers how the GUI must be set up when the headers are
   driven externally, and add it here.

TDD operation can also be controlled through the MCU, using the TX/rx_1_EXT and TX/rx_2_EXT interrupt signals on pins 7 and 8 of the SPI1 connector J28 (see :doc:`peripheralinterfaces`), or from the GUI.
