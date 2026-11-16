Peripheral Interfaces
#####################

The LimeRFE 8001P is controlled by the RP2040 microcontroller (MCU, U115). The MCU communicates with the host and other boards through the interfaces described below.

SPI1 Connector
**************

SPI1 is the external bus between the MCU and an external control board, such as LimeSDR-PCIe-5G. The MCU on the LimeRFE 8001P is the slave on the SPI1 bus.
The SPI1 bus is available on the 12-pin PMOD connector J28. The signals pass through the level shifters U24 and U163.
The allowed voltage range for this connector is 1.65 – 5.5 V. The voltage applied to pins 6 and 12 (VDD_CON_J28) sets the signal levels on the connector side of the level shifters.
J28 pinout is given in Table 10.

.. list-table:: Table 10. SPI1 connector (J28) pinout
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
     - VDD_CON_J28
     - Reference voltage input, must correspond to the SPI1 bus signal levels
   * - 7
     - TX/rx_1_EXT
     - Interrupt signal for the MCU, TDD mode control of channel 1
   * - 8
     - TX/rx_2_EXT
     - Interrupt signal for the MCU, TDD mode control of channel 2
   * - 9
     - MCU_GPIO18_EXT
     - MCU GPIO18 (not connected by default, R177 not fitted)
   * - 10
     - MCU_GPIO19_EXT
     - MCU GPIO19 (not connected by default, R178 not fitted)
   * - 11
     - GND
     - Ground
   * - 12
     - VDD_CON_J28
     - Reference voltage input, must correspond to the SPI1 bus signal levels

.. note::
   Please use a shielded data cable when connecting the LimeRFE 8001P to another board.

SPI2 Connector
**************

SPI2 is the internal bus between the MCU and the devices on the board (LMS8001B, SC1905 and ADF4002). The MCU is the master on the SPI2 bus.
The SPI2 bus is available on the 3x2 pin header J19. The allowed voltage for this connector is 3.3 V. J19 pinout is given in Table 11.

.. list-table:: Table 11. SPI2 connector (J19) pinout
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
     - NC
     - Not connected
   * - 6
     - GND
     - Ground

I2C Connector
*************

The I2C connector J1 (1x5 pin header) is intended for front panel control. The signals pass through the level shifters U153 and U154.
The voltage applied to pin 2 (VDD_CON_J1) sets the I2C signal levels and must be in the range 2.3 – 5.5 V. J1 pinout is given in Table 12.

.. list-table:: Table 12. I2C connector (J1) pinout
   :header-rows: 1

   * - Pin
     - Signal
   * - 1
     - GND
   * - 2
     - VDD_CON_J1
   * - 3
     - RPI_ID_SC_EXT
   * - 4
     - RPI_ID_SD_EXT
   * - 5
     - RPI_BTN2_EXT

USB
***

The board has two USB connection options:

* CON1 – micro USB type B connector. USB data lines are protected by U112.
* J29 – 2x5 USB pin header (pin 9 removed for keying), for connection to a motherboard.

J29 pinout is given in Table 13.

.. list-table:: Table 13. USB header (J29) pinout
   :header-rows: 1

   * - Pin
     - Signal
   * - 1, 2
     - VBUS_USB
   * - 3, 4
     - USB_DM1
   * - 5, 6
     - USB_DP1
   * - 7, 8
     - GND
   * - 10
     - NC

.. warning::
   The USB port cannot provide enough current for all RF devices. Do not use USB as the regular supply source, see :doc:`power`.

MCU Programming
***************

* J46 – boot selection header. Shorting J46 pulls the QSPI flash chip select low, and the MCU enters the BOOTSEL mode. In this mode the MCU appears to the host as external storage, and new firmware can be copied directly to it.
* SW2 – MCU reset switch.

The MCU firmware is stored in a 128 Mbit QSPI NOR flash (U114, AT25SF128A). The U113 footprint is an unfitted alternative. The MCU clock is a 12 MHz crystal (X6).
