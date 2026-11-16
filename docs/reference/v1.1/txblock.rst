Transmitter Block
#################

Each channel of the LimeRFE 8001P has a transmitter (TX) path, which linearizes the IF signal, converts it up to the RF band and amplifies it.
The description below is given for channel 1 (CH#1). Channel 2 (CH#2) is identical; the corresponding connectors are listed in brackets.

Connectors
**********

* J6, IFTX1 (J9, IFTX2) – IF input.
* J15, TXRX1 (J20, TXRX2) – RF output in TX mode.
* J18 and J37 (J23 and J40) – optional external TDD control of the driver amplifiers, see :doc:`gpios`.

.. warning::
   The maximum allowed input power at IFTX1 and IFTX2 depends on the AMP1 and AMP2 state, and is as low as –14 dBm when the amplifier is on. See :ref:`absolute-maximum-ratings`.

Sub-blocks
**********

Starting from the IFTX1 connector, the TX path of CH#1 consists of the following sub-blocks:

* Band-pass filter `DEA161953BT-2303B1-H`_, pass band 1805 – 2025 MHz, placed directly after the IFTX1 connector.
* IF amplifier (AMP1), with `TQL9063`_ as the main component. If the input signal level is high enough, AMP1 can be bypassed by software (BP_AMP1 bit) or by 0 |Omega| resistors.
* Linearization block, with `SC1905`_ as the main component, see :ref:`tx-linearization` below.
* Fixed 3 dB attenuator `PAT1220`_ (PAT1220-C-3DB-T5, U96). Footprints for 6 dB and 10 dB alternatives are not fitted.
* Band-pass filter `BFCN-1860+`_, pass band 1580 – 2200 MHz.
* Frequency conversion block, channel C (*Ch.C*) of the LMS8001B (U1), see :ref:`frequency-conversion`.
* Fixed 6 dB attenuator `PAT1220`_ (PAT1220-C-6DB-T5, U107), which improves the matching between the frequency conversion block and the predriver amplifier. Footprints for 3 dB and 10 dB alternatives are not fitted.
* Band-pass filter `BFCV-4085+`_, pass band 3100 – 4900 MHz.
* Predriver amplifier (PDA1), with `MNA-7A+`_ as the main component. PDA1 is enabled by the PDA1_EN bit.
* Digital step attenuator `F1956NBGI8`_ (TX1 Att.), 0 – 31.75 dB in steps of 0.25 dB.
* RF switch `SKY13330-397LF`_, which selects branch A or branch B of the driver amplifier.
* Driver amplifier (DA1), with two parallel branches:

  * branch A (DA1_A): `MMZ38333B`_,
  * branch B (DA1_B): fixed 4 dB attenuator (PAT1220-C-4DB-T5) and `QPA9122M`_.

  The branch is selected by the DA1_A_B_CTRL bit, and enabled by the DA1_A_EN or DA1_B_EN bit.
* RF switch `SKY13330-397LF`_, which combines branch A and branch B.
* RF switch `SKY13330-397LF`_, which selects the transmitter or receiver path. Signal path selection is controlled by the TXRX_1 bit.
* Directional coupler `HHM2955A1`_ at the DA output. The coupled signal can be returned through the ORX path to the SC1905, so that the on-board driver amplifier can be linearized.
* Band-pass filter `BFCV-4085+`_, pass band 3100 – 4900 MHz, which rejects all unwanted signals outside the RF band before the antenna or an external power amplifier.

Key TX characteristics of CH#1 are given in the Key Characteristics table on the :doc:`/index` page. The output 1 dB compression point at TXRX1 is 26 dBm with branch A and 22 dBm with branch B.

.. _tx-linearization:

Linearization
*************

The linearization block linearizes an amplifier, using the `SC1905`_ RF power amplifier linearizer.
It has two inputs (RFIN and RFFB) and one output (RFOUT):

* RFIN is the IF signal to be amplified.
* RFOUT is connected (through the frequency conversion block) to the input of the amplifier to be linearized.
* RFFB is the feedback signal from the amplifier output, returned to the board through the ORX path (see :doc:`orxblock`).

The SC1905 compares the shapes of the RFIN and RFFB signals, regardless of their amplitude, and adjusts the RFOUT signal so that the RFFB signal is as close as possible to the RFIN signal.
Besides the SC1905 chip, the linearization block contains capacitors, baluns, couplers, resistors, attenuators and a delay line.

The amplifier to be linearized can be:

* the on-board driver amplifier chain (PDA and DA), or
* an external power amplifier connected to the TXRX connector.

The linearization block can be turned off by software, or bypassed by fitting 0 |Omega| resistors (see :ref:`orx-reconfiguration`). Linearization supports FDD and TDD OFDM signals with bandwidths up to 100 MHz.

Signal Paths
************

.. list-table:: Table 4. TX configurations and corresponding IF and RF signal paths
   :header-rows: 1

   * - Channel
     - Signal path
   * - CH#1
     - J6 / DEA161953BT-2303B1-H / TQL9063 / [SC1905 (CH#1)] / PAT1220-C-3DB-T5 / BFCN-1860+ / LMS8001B (CH#1, *Ch.C*) /

       PAT1220-C-6DB-T5 / BFCV-4085+ / MNA-7A+ / F1956NBGI8 / SKY13330-397LF /

       (MMZ38333BT1) or (PAT1220-C-4DB-T5 and QPA9122M) /

       SKY13330-397LF / SKY13330-397LF / HHM2955A1 / BFCV-4085+ / J15
   * - CH#2
     - J9 / DEA161953BT-2303B1-H / TQL9063 / [SC1905 (CH#2)] / PAT1220-C-3DB-T5 / BFCN-1860+ / LMS8001B (CH#2, *Ch.C*) /

       PAT1220-C-6DB-T5 / BFCV-4085+ / MNA-7A+ / F1956NBGI8 / SKY13330-397LF /

       (MMZ38333BT1) or (PAT1220-C-4DB-T5 and QPA9122M) /

       SKY13330-397LF / SKY13330-397LF / HHM2955A1 / BFCV-4085+ / J20

.. note::
   Square brackets mark the linearization block, which can be bypassed.

.. _DEA161953BT-2303B1-H: https://product.tdk.com/en/search/rf/rf/filter/info?part_no=DEA161953BT-2303B1-H
.. _TQL9063: https://www.qorvo.com/products/p/TQL9063
.. _SC1905: https://www.maximintegrated.com/en/products/comms/wireless-rf/SC1905.html
.. _BFCN-1860+: https://www.minicircuits.com/WebStore/dashboard.html?model=BFCN-1860%2B
.. _PAT1220: https://www.susumu.co.jp/common/pdf/n_catalog_partition16_en.pdf
.. _BFCV-4085+: https://www.minicircuits.com/WebStore/dashboard.html?model=BFCV-4085%2B
.. _MNA-7A+: https://www.minicircuits.com/WebStore/dashboard.html?model=MNA-7A%2B
.. _F1956NBGI8: https://www.renesas.com/us/en/products/rf-products/rf-attenuators/f1956-7-bit-025db-glitch-free-wideband-digital-step-attenuator
.. _SKY13330-397LF: https://www.skyworksinc.com/en/Products/switches/SKY13330-397LF
.. _MMZ38333B: https://www.nxp.com/part/MMZ38333B
.. _QPA9122M: https://www.qorvo.com/products/p/QPA9122M
.. _HHM2955A1: https://product.tdk.com/en/search/rf/rf/coupler/info?part_no=HHM2955A1
