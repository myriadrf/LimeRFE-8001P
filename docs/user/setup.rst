Hardware Setup
##############

Host Interface
**************

There are 2 possible ways to control the LimeRFE 8001P board:

* Through a master control board, such as LimeSDR-PCIe-5G – communication is established over the SPI interface, with the master control board connected to the J28 connector. In this documentation this type of communication is denoted SDR.
* Directly through USB – using the micro USB connector (CON1).

See :doc:`software/setup` for the connection procedure.

Cooling
*******

The LimeRFE 8001P board ships with a pre-installed heatsink. Under normal ambient temperature conditions, passive cooling is sufficient to maintain stable and reliable board operation.

Default heatsink is `Wakefield Thermal 558-75AB`_:

* Mounting style: screw.
* Heatsink material: aluminium.
* Fin style: rectangular fin.
* Thermal resistance: 1.8 °C/W.
* Power dissipation at temperature rise: 12.0 W at 40 °C.
* Length: 61 mm.
* Width: 116.8 mm.
* Height: 19.1 mm.

.. note::
   In the event of errors, instability or reduced performance, check the board temperature to ensure that it is within the specified operating range.

Power Connections
*****************

The board can be supplied from:

* an external 5 V power supply, connected to J2,
* an external 12 V power supply, connected to J27, J30 or J45 (an on-board DC-DC converter generates 5 V),
* USB (CON1). This option is not recommended, because USB cannot provide enough current for the RF devices.

The RF devices (amplifiers) have a separate 5 V supply selection: they can be supplied from the board 5 V supply, from USB, or from an external 5 V power supply connected to J24.
The supply sources are selected with jumpers on the JP1 (board supply) and JP2 (RF devices supply) headers, shown in Figure 2.

.. figure:: /images/LimeRFE-8001P_v1.1_prw_con.png
  :width: 600

  Figure 2. LimeRFE 8001P v1.1 power supply connectors and source selection headers

.. warning::
   Do not place more than one jumper on each of the JP1 and JP2 headers.

For the jumper positions and recommended configurations, see :doc:`/reference/v1.1/power`.



RF Connections
**************

.. figure:: /images/LimeRFE-8001P_v1.1_rfcons.png
  :width: 600
  
  Figure 3: LimeRFE 8001P board top with RF connector positions

.. list-table:: Table 1. RF Connectors
   :header-rows: 1

   * - Connector
     - Designation
     - Description
   * - J6
     - IF_TX1
     - IF frequency band (central frequency 1900 MHz), TX input connector, Channel 1
   * - J7
     - IF_ORX1
     - IF frequency band (central frequency 1900 MHz), observation output connector, Channel 1
   * - J8
     - IF_RX1
     - IF frequency band (central frequency 1900 MHz), RX output connector, Channel 1
   * - J9
     - IF_TX2
     - IF frequency band (central frequency 1900 MHz), TX input connector, Channel 2
   * - J10
     - IF_ORX2
     - IF frequency band (central frequency 1900 MHz), observation output connector, Channel 2
   * - J11
     - IF_RX2
     - IF frequency band (central frequency 1900 MHz), RX output connector, Channel 2
   * - J15
     - TXRX1
     - RF frequency band (3100 – 4900 MHz), TXRX antenna connector, Channel 1
   * - J4
     - ORX1
     - RF frequency band (3100 – 4900 MHz), observation input connector, Channel 1
   * - J20
     - TXRX2
     - RF frequency band (3100 – 4900 MHz), TXRX antenna connector, Channel 2
   * - J5
     - ORX2
     - RF frequency band (3100 – 4900 MHz), observation input connector, Channel 2


.. warning::
   Care should be taken when connecting external RF signals to the board, to ensure that the maximum allowed input power is not exceeded, as this may cause permanent damage to the device.
   For example, the maximum input power at TXRX1 and TXRX2 in RX mode is +8 dBm, and at IFTX1 and IFTX2 it is as low as –14 dBm when AMP1 or AMP2 is on.
   See :ref:`absolute-maximum-ratings` for all connectors.

.. _Wakefield Thermal 558-75AB: https://wakefieldthermal.com/wakefield-thermal-heatsink-dc-dc-full-brick-558-75ab/




