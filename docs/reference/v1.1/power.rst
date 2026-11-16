Power Supply
############

The LimeRFE-8001P board has multiple power supply source options. The power supply option is set by configuring connectors JP1 and JP2 and properly setting up the connectors J2, J24, J27, J30, J45, J16 and J21. All of these connectors are placed on the top side of the board, grouped as shown in Figure 9. CON1 is a micro USB type B connector and could be used for the LimeRFE 8001P board supply. However, due to the USB port current limitation, it is not recommended to use this as regular DC supply source, particularly when higher power devices are turned on (PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, any of two LMS8001Bs or any of two SC1905s). Connectors J16 and J21 are only for RF devices supply and should be used only for test and debug purposes.


.. figure:: /images/LimeRFE-8001P_v1.1_prw_con.png
  :width: 600

  Figure 9. LimeRFE 8001P v1.1 power supply connectors and source selection headers

Options
=======

There are several ways of providing 5 V supply to the board and independently several ways of providing 5 V supply to the RF devices: AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1 and LNA2). It is possible to select one of numerous options by placing jumpers on connectors JP1 and JP2, see Table 14.

.. list-table:: Table 14. Power supply options
   :header-rows: 1

   * - Connector
     - Selected 
      
       option
     - Position
     - Comments
   * - JP1
     - Bottom
     - (Pins 5-6) 
      
       5 V from external power supply
     - 5 V is obtained from J2.
   * - JP1
     - Center
     - (Pins 3-4)
      
       5 V from USB
     - 5 V is obtained from USB (CON1).
   * - JP1
     - Top
     - (Pins 1-2)
      
       5 V from external 12 V power supply
     - 5 V is obtained from DC–DC converter which is supplied from 12 V.
      
       12 V is obtained from J27, J30 or J45.
   * - JP2
     - Bottom
     - (Pins 5-6)
      
       5 V from external power supply
     - 5 V for RF devices are obtained from J24 connector.
   * - JP2
     - Center
     - (Pins 3-4)
      
       5 V from USB
     - 5 V for RF devices are obtained from USB (CON1). 
      
       **Should be avoided!**
   * - JP2
     - Top
     - (Pins 1-2) 5 V from the board
     - 5 V is obtained from the board. For the board 5 V source, 
      
       please see the JP1 jumper position in this table. 
  
       **Avoid 5V from USB option in JP1** 
       


.. warning::
  Do not place more than one jumper per JP1 and JP2 connector.
  
  Standard USB 2.0 5V cannot provide enough power to supply RF amplifiers.

Preferred configuration of JP1 connector is at jumper position:

* bottom (Pins 5-6), where 5 V is obtained from the external 5 V power supply (J2), or
* top (Pins 1-2), where 5 V is obtained from DC-DC converter which is supplied from external 12 V power supply (J27, J30 or J45).

Preferred configuration of JP2 connector is at jumper position:

* bottom (Pins 5-6), where 5 V is obtained from the external 5 V power supply (J24), or
* top (Pins 1-2), where 5 V is obtained from the LimeRFE 8001P board.

DC Current Consumption
======================

A list of the main devices power consumption, as well as their default bias currents are given in Table 15.

.. list-table:: Table 15. DC current consumption
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
     - A brief description about control bits and how to turn on is given in 
      
       Table 6. A description of AMP1 or/and AMP2 is given in Table 7.
   * - PDA1 / PDA2
     - ~90 mA
     - ~50 mA
     - A brief description about control bits is given in Table 6. A description 
     
       how to turn on PDA1 or/and PDA2 is given in Table 7.
   * - DA1_A / DA2_A
     - ~340 mA
     - ~150 mA
     - A brief description about control bits is given in Table 6. A description 
     
       how to turn on DA1_A or/and DA2_A is given in Table 7.
   * - DA1_B / DA2_B
     - ~160 mA
     - ~105 mA
     - A brief description about control bits is given in Table 6. A description 
    
       how to turn on DA1_B or/and DA2_B is given in Table 7.
   * - LNA1 / LNA2
     - ~35 mA
     - ~20 mA
     - A brief description about control bits is given in Table 6. A description 
      
       how to turn on LNA1 or/and LNA2 is given in Table 7.
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
    All currents are approximate values, but care should be taken if the actual values differ significantly from the ones given in the table above, Table 15. Jumpers on connectors JP1 and JP2 must be properly placed (for a detailed description refer to Table 14).

Figure 10 shows the main DC current consumption devices (AMP1, AMP2, PDA1, PDA2, DA1_A, DA1_B, DA2_A, DA2_B, LNA1, LNA2, SC1905_#1, SC1905_#2, LMS8001B_#1, LMS8001B_#2).

.. figure:: /images/LimeRFE-8001P_v1.1_prw_dc.png
  :width: 600

  Figure 10. The main DC current consumption devices in the LimeRFE 8001P board