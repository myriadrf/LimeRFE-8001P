Peripheral Interfaces
#####################

The LimeRFE 8001P is controlled by the RP2040 microcontroller (MCU, U115). The MCU communicates with the host and other boards through the interfaces described below.

SPI1 Connector
==============

SPI1 is the external bus between the MCU and an external control board, such as LimeSDR-PCIe-5G. The MCU on the LimeRFE 8001P is the slave on the SPI1 bus.
The SPI1 bus is available on the 12-pin PMOD connector J28. The signals pass through the level shifter U24. J28 pinout is given in Table 11.

.. list-table:: Table 11. SPI1 connector (J28) pinout
   :header-rows: 1

   * - Pin
     - Signal
     - Description
   * - 1
     - SS_1_EXT
     - Slave select
   * - 2
     - MOSI_1_EXT
     - Master output, slave input
   * - 3
     - MISO_1_EXT
     - Master input, slave output
   * - 4
     - SCLK_1_EXT
     - Clock signal
   * - 5
     - GND
     - Ground
   * - 6
     - Supply
     - External power supply, must correspond to the SPI1 bus signal levels
   * - 7
     - TX/rx_1_EXT
     - Interrupt signal for the MCU, TDD mode control of channel 1
   * - 8
     - TX/rx_2_EXT
     - Interrupt signal for the MCU, TDD mode control of channel 2
   * - 9
     - NC
     - Not connected in the default configuration (R177 not fitted)
   * - 10
     - NC
     - Not connected in the default configuration (R178 not fitted)
   * - 11
     - GND
     - Ground
   * - 12
     - Supply
     - External power supply, must correspond to the SPI1 bus signal levels

.. note::
   Please use a shielded data cable when connecting the LimeRFE 8001P to another board.

SPI2 Connector
==============

SPI2 is the internal bus between the MCU and the devices on the board (LMS8001B, SC1905 and ADF4002). The MCU is the master on the SPI2 bus.
The SPI2 bus is available on the 3x2 pin header J19. The allowed voltage for this connector is 3.3 V. J19 pinout is given in Table 12.

.. list-table:: Table 12. SPI2 connector (J19) pinout
   :header-rows: 1

   * - Pin
     - Signal
     - Description
   * - 1
     - MISO_2
     - Master input, slave output
   * - 2
     - VDIO
     - 3.3 V DC supply from the board
   * - 3
     - SCLK_2
     - Clock signal
   * - 4
     - MOSI_2
     - Master output, slave input
   * - 5
     - RESET
     - MCU reset
   * - 6
     - GND
     - Ground

.. Note: the source manual gives pin 5 of J19 as not connected. Both the v1.0 and v1.1 netlists connect
   pin 5 to the MCU RESET net.

SC1905 Debug Connectors
=======================

The SC1905 linearizers can be controlled directly by a Maxim micro-controller board and the Maxim GUI software, for debugging. The 14-pin connectors J32 (SC1905 on channel 1) and J48 (SC1905 on channel 2) are provided for this purpose.

By default, the SC1905 chips are connected to the board MCU through the SPI2 bus. This connection is made through the bus switches U133 (channel 1) and U139 (channel 2), which are enabled by jumpers on J53 (channel 1) and J55 (channel 2):

* jumper placed – the SC1905 is connected to the board MCU,
* jumper removed – the SC1905 is disconnected from the SPI2 bus.

.. warning::
   Remove the jumper from J53 (J55) before connecting a Maxim micro-controller board to J32 (J48).

J32 and J48 pinout is given in Table 13.

.. list-table:: Table 13. SC1905 debug connectors (J32, J48) pinout
   :header-rows: 1

   * - Pin
     - Signal
   * - 1
     - WDTEN
   * - 2
     - NC
   * - 3
     - LOADENB
   * - 4
     - STATO
   * - 5
     - DGPIN1
   * - 6
     - RESETN
   * - 7
     - DGPIN0
   * - 8
     - SSN
   * - 9
     - GND
   * - 10
     - SDI
   * - 11
     - GND
   * - 12
     - SDO
   * - 13
     - GND
   * - 14
     - SCLK

The digital signals of the SC1905 chips are also available on the 12-pin PMOD connectors J52 (channel 1) and J51 (channel 2), through level shifters, so that signal levels other than 3.3 V can be used.
These connectors are not protected by the J53 and J55 jumpers, so care must be taken to avoid signal collisions on the SPI2 bus. J52 and J51 pinout is given in Table 14.

.. list-table:: Table 14. SC1905 PMOD connectors (J52, J51) pinout
   :header-rows: 1

   * - Pin
     - Signal
     - Description
   * - 1
     - STATO_EXT
     - SC1905 STATO
   * - 2
     - SSN_EXT
     - SC1905 SSN
   * - 3
     - SDI_EXT
     - SC1905 SDI
   * - 4
     - SDO_EXT
     - SC1905 SDO
   * - 5
     - GND
     - Ground
   * - 6
     - Supply
     - Default: DC supply input, must be in the range 3.3 – 5 V (R474 / R543 fitted, R473 / R503 not fitted).

       Option: VDIO (3.3 V) from the board (R474 / R543 not fitted, R473 / R503 fitted).
   * - 7
     - SCLK_EXT
     - SC1905 SCLK
   * - 8
     - WDTEN_EXT
     - SC1905 WDTEN
   * - 9
     - LOADENB_EXT
     - SC1905 LOADENB
   * - 10
     - DGPIN1_EXT
     - SC1905 DGPIN1
   * - 11
     - GND
     - Ground
   * - 12
     - Supply
     - Default: DC supply input, must be in the range 3.3 – 5 V (R488 / R546 fitted, R489 / R552 not fitted).

       Option: VDIO (3.3 V) from the board (R488 / R546 not fitted, R489 / R552 fitted).

In the resistor lists above, the first designator applies to J52 (channel 1) and the second to J51 (channel 2).

USB
===

CON1 is a micro USB type B connector. USB data lines are protected by U112.

.. warning::
   The USB port cannot provide enough current for all RF devices. Do not use USB as the regular supply source, see :doc:`power`.

MCU Programming
===============

* J46 – boot selection header. When J46 is shorted, the MCU enters the BOOTSEL mode. In this mode the MCU appears to the host as external storage, and new firmware can be copied directly to it.
* SW2 – MCU reset switch.
