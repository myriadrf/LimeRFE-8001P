Power Supply
############

The LimeRFE 8001P board has multiple power supply source options. The power supply source is selected with jumpers on JP1 and JP2, and the supply is connected to the corresponding connector.
The power supply connectors and source selection headers are listed in Table 14. All of them are placed on the top side of the board, grouped as shown in Figure 9.

.. figure:: /images/LimeRFE-8001P_v1.1_prw_con.png
  :width: 600

  Figure 9: LimeRFE 8001P v1.1 power supply connectors and source selection headers

.. list-table:: Table 14. Power supply connectors and source selection headers
   :header-rows: 1

   * - Connector
     - Type
     - Description
   * - J2
     - DC input
     - External 5 V supply for the board (JP1 in bottom position).
   * - J27
     - DC input, 3 pin header
     - External 12 V supply for the on-board DC-DC converter (JP1 in top position).
   * - J30
     - DC input, barrel jack
     - External 12 V supply for the on-board DC-DC converter (JP1 in top position).
   * - J45
     - DC input, 2x3 pin power connector
     - External 12 V supply for the on-board DC-DC converter (JP1 in top position).
   * - J24
     - DC input
     - External 5 V supply for the RF devices (JP2 in bottom position).
   * - J16, J21
     - DC input
     - Additional 5 V supply (+5V_RF) for the RF devices.

       For test and debug purposes only.
   * - CON1
     - Micro USB type B
     - USB 5 V supply for the board or the RF devices (JP1 or JP2 in center position).

       Not recommended as a regular supply source, due to the USB port current limitation, particularly when

       higher power devices are turned on (PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LMS8001Bs or SC1905s).
   * - JP1
     - Jumper header
     - Board 5 V supply source selection, see Table 15.
   * - JP2
     - Jumper header
     - RF devices 5 V supply source selection, see Table 15.

Options
=======

There are several ways of providing 5 V supply to the board and independently several ways of providing 5 V supply to the RF devices: AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1 and LNA2). It is possible to select one of numerous options by placing jumpers on connectors JP1 and JP2, see Table 15.

.. table:: Table 15. Power supply options

  +-----------+-----------------+-------------------------------------+----------------------------------------------------------------------+
  | Connector | Selected option | Position                            | Comments                                                             |
  +===========+=================+=====================================+======================================================================+
  | JP1       | Top             | (Pins 1-2)                          | 5 V is obtained from DC–DC converter which is supplied from 12 V.    |
  |           |                 |                                     |                                                                      |
  |           |                 | 5 V from external 12 V power supply | 12 V is obtained from J27, J30 or J45.                               |
  |           +-----------------+-------------------------------------+----------------------------------------------------------------------+
  |           | Center          | (Pins 3-4)                          | 5 V is obtained from USB (CON1).                                     |
  |           |                 |                                     |                                                                      |
  |           |                 | 5 V from USB                        |                                                                      |
  |           +-----------------+-------------------------------------+----------------------------------------------------------------------+
  |           | Bottom          | (Pins 5-6)                          | 5 V is obtained from J2.                                             |
  |           |                 |                                     |                                                                      |
  |           |                 | 5 V from external power supply      |                                                                      |
  +-----------+-----------------+-------------------------------------+----------------------------------------------------------------------+
  | JP2       | Top             | (Pins 1-2)                          | 5 V is obtained from the board.                                      |
  |           |                 |                                     |                                                                      |
  |           |                 | 5 V from the board                  | For the board 5 V source, see the JP1 jumper position in this table. |
  |           |                 |                                     |                                                                      |
  |           |                 |                                     | **Avoid 5V from USB option in JP1**                                  |
  |           +-----------------+-------------------------------------+----------------------------------------------------------------------+
  |           | Center          | (Pins 3-4)                          | 5 V for RF devices is obtained from USB (CON1).                      |
  |           |                 |                                     |                                                                      |
  |           |                 | 5 V from USB                        | **Should be avoided!**                                               |
  |           +-----------------+-------------------------------------+----------------------------------------------------------------------+
  |           | Bottom          | (Pins 5-6)                          | 5 V for RF devices is obtained from J24 connector.                   |
  |           |                 |                                     |                                                                      |
  |           |                 | 5 V from external power supply      |                                                                      |
  +-----------+-----------------+-------------------------------------+----------------------------------------------------------------------+

.. warning::
  Do not place more than one jumper per JP1 and JP2 connector.
  
  Standard USB 2.0 5V cannot provide enough power to supply RF amplifiers.

Preferred Configuration with 12 V Supply
----------------------------------------

* JP1 in top position (Pins 1-2) – the board 5 V is obtained from the on-board DC-DC converter, which is supplied from the external 12 V power supply (J27, J30 or J45).
* JP2 in top position (Pins 1-2) – the RF devices are supplied from the board 5 V.

Preferred Configuration with 5 V Supply
---------------------------------------

* JP1 in bottom position (Pins 5-6) – the board 5 V is obtained from the external 5 V power supply (J2).
* JP2 in top position (Pins 1-2) – the RF devices are supplied from the board 5 V, or
  JP2 in bottom position (Pins 5-6) – the RF devices are supplied from a separate external 5 V power supply (J24).

DC Current Consumption
======================

A list of the main devices power consumption, as well as their default bias currents are given in Table 16.

.. list-table:: Table 16. DC current consumption
   :header-rows: 1

   * - Device
     - Bias current @5 V
     - Bias current @12 V
     - Comments
   * - Reset state
     - ~230 mA
     - ~155 mA
     -
   * - AMP1 / AMP2
     - ~80 mA
     - ~50 mA
     -
   * - PDA1 / PDA2
     - ~90 mA
     - ~50 mA
     -
   * - DA1_A / DA2_A
     - ~340 mA
     - ~150 mA
     -
   * - DA1_B / DA2_B
     - ~160 mA
     - ~105 mA
     -
   * - LNA1 / LNA2
     - ~35 mA
     - ~20 mA
     -
   * - SC1905 (CH#1) / SC1905 (CH#2)
     - ~500 mA
     - ~350 mA
     - During different phases in SC1905 operation ("INIT", "CAL", "PDET", 
      
       "FSA" and "TRACK"), current consumption can differ, but not more than 
       
       20%. Typically, current consumption changes in a range of 
       
       400–550 mA. After entering "TRACK" mode, current consumption 
       
       should be steady.
   * - LMS8001B (CH#1) / LMS8001B (CH#2)
     - ~240 mA
     - ~225 mA
     - This is the case with Ch.B, Ch.C and Ch.D activated. 
       
       DC current consumption of the LMS8001B can be significantly different, 
       
       and special care must be taken to ensure the LMS8001B works properly.

.. note::
    All currents are approximate values, but care should be taken if the actual values differ significantly from the ones given in Table 16. Jumpers on connectors JP1 and JP2 must be properly placed (for a detailed description refer to Table 15).

Figure 10 shows the main DC current consumption devices (AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1, LNA2, SC1905_#1, SC1905_#2, LMS8001B_#1, LMS8001B_#2).

.. figure:: /images/LimeRFE-8001P_v1.1_prw_dc.png
  :width: 600

  Figure 10: The main DC current consumption devices in the LimeRFE 8001P board