Observation Block
#################

Each channel of the LimeRFE 8001P has an observation (ORX) path. It takes a sample of the amplifier output signal, attenuates it and converts it down to the IF band.
By default, the ORX signal is used as the feedback (RFFB) signal for the SC1905 linearizer (see :ref:`tx-linearization`).

The description below is given for channel 1 (CH#1). Channel 2 (CH#2) is identical; the corresponding connectors are listed in brackets.

Connectors
**********

* J4, ORX1 (J5, ORX2) – RF input. The signal is taken from the output of an external amplifier through a directional coupler.
* J7, IFORX1 (J10, IFORX2) – optional IF output. It is not used in the default configuration (see :ref:`orx-reconfiguration`).

.. warning::
   The maximum allowed input power at ORX1 and ORX2 is +37 dBm when the ORX attenuation is set above 23 dB. With lower attenuation, the LMS8001B input (+4 dBm maximum) becomes the limiting component. See :ref:`absolute-maximum-ratings`.

Sub-blocks
**********

Starting from the ORX1 connector, the ORX path of CH#1 consists of the following sub-blocks:

* Fixed 10 dB attenuator `PAT3060H-C-10DB-T1`_, which attenuates the signal from the external power amplifier.
* Digital step attenuator `F1956NBGI8`_ (ORX1 Att.), 0 – 31.75 dB in steps of 0.25 dB.
* Frequency conversion block, channel D (*Ch.D*) of the LMS8001B (U1), see :ref:`frequency-conversion`.
* In the default configuration, the *Ch.D* output is routed to the RFFB input of the SC1905.
  If the board is reconfigured with 0 |Omega| resistors, the signal is routed instead through the band-pass filter `BFCN-1860+`_ (pass band 1580 – 2200 MHz) to the IFORX1 connector.

Instead of the ORX1 connector, the ORX path can take its input from the on-board directional coupler `HHM2955A1`_ at the driver amplifier output.
This option is selected with 0 |Omega| resistors (see :ref:`orx-reconfiguration`).

The gain from ORX1 to IFORX1 is –17 dB, with ORX1 Att. set to 0 dB.

Signal Paths
************

The ORX path options are:

* A – external coupler, ORX connector, signal routed to the SC1905 (default),
* B – external coupler, ORX connector, signal routed to the IFORX connector (linearization outside of the board),
* C – on-board coupler, signal routed to the SC1905,
* D – on-board coupler, signal routed to the IFORX connector (linearization outside of the board).

.. list-table:: Table 5. ORX configurations and corresponding RF and IF signal paths
   :header-rows: 1

   * - Channel
     - Option
     - Signal path
   * - CH#1
     - A
     - J4 / PAT3060H-C-10DB-T1 / F1956NBGI8 / LMS8001B (CH#1, *Ch.D*) / BFCN-1860+ / SC1905 (CH#1)
   * - CH#1
     - B
     - J4 / PAT3060H-C-10DB-T1 / F1956NBGI8 / LMS8001B (CH#1, *Ch.D*) / BFCN-1860+ / J7
   * - CH#1
     - C
     - HHM2955A1 / PAT3060H-C-10DB-T1 / F1956NBGI8 / LMS8001B (CH#1, *Ch.D*) / BFCN-1860+ / SC1905 (CH#1)
   * - CH#1
     - D
     - HHM2955A1 / PAT3060H-C-10DB-T1 / F1956NBGI8 / LMS8001B (CH#1, *Ch.D*) / BFCN-1860+ / J7
   * - CH#2
     - A
     - J5 / PAT3060H-C-10DB-T1 / F1956NBGI8 / LMS8001B (CH#2, *Ch.D*) / BFCN-1860+ / SC1905 (CH#2)
   * - CH#2
     - B
     - J5 / PAT3060H-C-10DB-T1 / F1956NBGI8 / LMS8001B (CH#2, *Ch.D*) / BFCN-1860+ / J10
   * - CH#2
     - C
     - HHM2955A1 / PAT3060H-C-10DB-T1 / F1956NBGI8 / LMS8001B (CH#2, *Ch.D*) / BFCN-1860+ / SC1905 (CH#2)
   * - CH#2
     - D
     - HHM2955A1 / PAT3060H-C-10DB-T1 / F1956NBGI8 / LMS8001B (CH#2, *Ch.D*) / BFCN-1860+ / J10

.. _orx-reconfiguration:

Reconfiguration
***************

The board functionality can be changed by fitting or removing 0 |Omega| resistors or small impedance capacitors at several locations.
The options are described in Table 6. Fitted components are marked F, components that are not fitted are marked NF, and the default state is marked DS.

.. list-table:: Table 6. Board reconfiguration with 0 Ohm resistors and small impedance capacitors
   :header-rows: 1

   * - Description of functionality
     - Channel 1 configuration
     - Channel 2 configuration
   * - SC1905 included in the IF path (linearization performed on the board).

       External PA is linearized.

       The PA output is coupled by an external coupler and returned through the ORX connector.
     - DS:

       R263 F, R262 NF

       R264 F, R265 NF

       R260 F, R261 NF

       R193 F, R191 NF
     - DS:

       R269 F, R268 NF

       R270 F, R271 NF

       R266 F, R267 NF

       R194 F, R192 NF
   * - SC1905 included in the IF path (linearization performed on the board).

       On-board PDA and DA are linearized.

       The DA output is coupled by an external coupler and returned through the ORX connector.
     - DS:

       R263 F, R262 NF

       R264 F, R265 NF

       R260 F, R261 NF

       R193 F, R191 NF
     - DS:

       R269 F, R268 NF

       R270 F, R271 NF

       R266 F, R267 NF

       R194 F, R192 NF
   * - SC1905 included in the IF path (linearization performed on the board).

       On-board PDA and DA are linearized.

       The DA output is coupled by the on-board coupler.
     - R263 F, R262 NF

       R264 F, R265 NF

       R260 F, R261 NF

       R193 NF, R191 F
     - R269 F, R268 NF

       R270 F, R271 NF

       R266 F, R267 NF

       R194 NF, R192 F
   * - SC1905 excluded from the IF path (linearization performed outside of the board).

       External PA is linearized.

       The PA output is coupled by an external coupler and returned through the ORX connector.

       The signal is output through the IFORX connector.
     - R263 NF, R262 F

       R264 NF, R265 F

       R260 NF, R261 F

       R193 F, R191 NF
     - R269 NF, R268 F

       R270 NF, R271 F

       R266 NF, R267 F

       R194 F, R192 NF
   * - SC1905 excluded from the IF path (linearization performed outside of the board).

       On-board PDA and DA are linearized.

       The DA output is coupled by an external coupler and returned through the ORX connector.

       The signal is output through the IFORX connector.
     - R263 NF, R262 F

       R264 NF, R265 F

       R260 NF, R261 F

       R193 F, R191 NF
     - R269 NF, R268 F

       R270 NF, R271 F

       R266 NF, R267 F

       R194 F, R192 NF
   * - SC1905 excluded from the IF path (linearization performed outside of the board).

       On-board PDA and DA are linearized.

       The DA output is coupled by the on-board coupler.

       The signal is output through the IFORX connector.
     - R263 NF, R262 F

       R264 NF, R265 F

       R260 NF, R261 F

       R193 NF, R191 F
     - R269 NF, R268 F

       R270 NF, R271 F

       R266 NF, R267 F

       R194 NF, R192 F
   * - Standard TXRX port used for transmitter and receiver.
     - DS:

       C233 F, R565 NF
     - DS:

       C328 F, R572 NF
   * - Dedicated RX port used for receiver only.
     - C233 NF, R565 F
     - C328 NF, R572 F
   * - Delay component (U116, U125) included in the SC1905 circuit.
     - DS:

       R364 F, R370 NF

       R363 F, R369 NF
     - DS:

       R427 F, R435 NF

       R426 F, R434 NF
   * - Delay component (U116, U125) bypassed in the SC1905 circuit.
     - R364 NF, R370 F

       R363 NF, R369 F
     - R427 NF, R435 F

       R426 NF, R434 F

.. _PAT3060H-C-10DB-T1: https://www.susumu.co.jp/common/pdf/n_catalog_partition30_en.pdf
.. _F1956NBGI8: https://www.renesas.com/us/en/products/rf-products/rf-attenuators/f1956-7-bit-025db-glitch-free-wideband-digital-step-attenuator
.. _BFCN-1860+: https://www.minicircuits.com/WebStore/dashboard.html?model=BFCN-1860%2B
.. _HHM2955A1: https://product.tdk.com/en/search/rf/rf/coupler/info?part_no=HHM2955A1
