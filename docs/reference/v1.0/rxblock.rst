Receiver Block
##############

Each channel of the LimeRFE 8001P has a receiver (RX) path, which amplifies the received RF signal and converts it down to the IF band.
The description below is given for channel 1 (CH#1). Channel 2 (CH#2) is identical; the corresponding connectors are listed in brackets.

Connectors
==========

* J15, TXRX1 (J20, TXRX2) – RF input in RX mode. This is the default RX input.
* J17, RX1 (J22, RX2) – optional dedicated RF input. It is not used in the default configuration (see :ref:`orx-reconfiguration-v1.0`).
* J8, IFRX1 (J11, IFRX2) – IF output.

.. warning::
   The maximum allowed input power at TXRX1 and TXRX2 in RX mode is +8 dBm (LNA on). See :ref:`absolute-maximum-ratings`.

Sub-blocks
==========

Starting from the TXRX1 connector, the RX path of CH#1 consists of the following sub-blocks:

* Band-pass filter `BFCV-4085+`_, pass band 3100 – 4900 MHz.
* RF switches `SKY13330-397LF`_, which select the transmitter or receiver path. Signal path selection is controlled by the TXRX_1 bit.
* Low noise amplifier (LNA1), with `TSS-53LNB3+`_ as the main component. LNA1 is enabled by the LNA1_EN bit.
* Frequency conversion block, channel D (*Ch.D*) of the LMS8001B (U1), see :ref:`frequency-conversion-v1.0`.
* Band-pass filter `DEA161953BT-2303B1-H`_, pass band 1805 – 2025 MHz, which rejects all signals outside the IF band.

The dedicated RX1 input is connected through its own `BFCV-4085+`_ band-pass filter. RX1 is not on the edge of the board, and this path is shown with a dashed line in the block diagram.
It can be activated by fitting 0 |Omega| resistors (see :ref:`orx-reconfiguration-v1.0`).

Key RX characteristics of CH#1 are:

* Gain from TXRX1 to IFRX1: 7 dB.
* Gain from RX1 to IFRX1: 7 dB.

Signal Paths
============

.. list-table:: Table 3. RX configurations and corresponding RF and IF signal paths
   :header-rows: 1

   * - Channel
     - Input
     - Signal path
   * - CH#1
     - TXRX1 (J15)

       Default
     - J15 / BFCV-4085+ / SKY13330-397LF / SKY13330-397LF / TSS-53LNB3+ /

       LMS8001B (CH#1, *Ch.D*) / DEA161953BT-2303B1-H / J8
   * - CH#1
     - RX1 (J17)
     - J17 / BFCV-4085+ / SKY13330-397LF / TSS-53LNB3+ /

       LMS8001B (CH#1, *Ch.D*) / DEA161953BT-2303B1-H / J8
   * - CH#2
     - TXRX2 (J20)

       Default
     - J20 / BFCV-4085+ / SKY13330-397LF / SKY13330-397LF / TSS-53LNB3+ /

       LMS8001B (CH#2, *Ch.D*) / DEA161953BT-2303B1-H / J11
   * - CH#2
     - RX2 (J22)
     - J22 / BFCV-4085+ / SKY13330-397LF / TSS-53LNB3+ /

       LMS8001B (CH#2, *Ch.D*) / DEA161953BT-2303B1-H / J11

.. _BFCV-4085+: https://www.minicircuits.com/WebStore/dashboard.html?model=BFCV-4085%2B
.. _SKY13330-397LF: https://www.skyworksinc.com/en/Products/switches/SKY13330-397LF
.. _TSS-53LNB3+: https://www.minicircuits.com/WebStore/dashboard.html?model=TSS-53LNB3%2B
.. _DEA161953BT-2303B1-H: https://product.tdk.com/en/search/rf/rf/filter/info?part_no=DEA161953BT-2303B1-H
