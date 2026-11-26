Hardware Setup
##############

Host Interface
==============

The LimeRFE 8001P board is controlled from the host PC through USB, using the micro USB connector (CON1).

See :doc:`software/setup` for the connection procedure.

Cooling
=======

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

.. _power-connections:

Power Connections
=================

The recommended power supply option is an external 12 V power supply:

#. Connect the 12 V power supply to J45 or to the barrel connector J30 (v1.1 only).
#. Place the JP1 jumper on pins 1-2 (top position). The board is then supplied from the DC-DC converter.
#. Place the JP2 jumper on pins 1-2 (top position). The RF devices (amplifiers) are then supplied from the board 5 V supply.

The power supply connectors and the JP1 and JP2 headers are shown in Figure 3. The jumper positions are the same on both board versions.

.. tabs::

   .. tab:: v1.1

      .. figure:: /images/LimeRFE-8001P_v1.1_prw_con.png
         :width: 200

         Figure 3: LimeRFE 8001P v1.1 power supply connectors and source selection headers

   .. tab:: v1.0

      .. figure:: /images/LimeRFE-8001P_v1.0_prw_con.png

         Figure 3: LimeRFE 8001P v1.0 power supply connectors and source selection headers

.. warning::
   Do not place more than one jumper on each of the JP1 and JP2 headers.

   Do not use USB (CON1) as the power supply. USB cannot provide enough current for the RF devices.

For all other power supply options, see :doc:`/reference/v1.1/power` (v1.1) or :doc:`/reference/v1.0/power` (v1.0).



RF Connections
==============

.. tabs::

   .. tab:: v1.1

      .. figure:: /images/LimeRFE-8001P_v1.1_rfcons.png
         :width: 600

         Figure 4: LimeRFE 8001P v1.1 board top with RF connector positions

   .. tab:: v1.0

      .. figure:: /images/LimeRFE-8001P_v1.0_rfcons.png
         :width: 600

         Figure 4: LimeRFE 8001P v1.0 board top with RF connector positions

The RF connectors have the same designators on both board versions.

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




