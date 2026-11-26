GUI
###

The main control window of the LimeRFE 8001P GUI is shown in Figure 5.

.. figure:: /images/LimeRFE-8001P_GUI.png
  :width: 600

  Figure 5: LimeRFE 8001P GUI main control window – all panels described

The **Main** tab (upper left corner of the window) is active during setup of the board. Below this tab, the following panels are visible:

* **Communication** – selects the connection type. Select **Direct (USB)**.
* **USB** and **Port** – select and open the serial port (``/dev/ttyACM*``) for the direct USB connection.
* **Configuration** – saves and loads the board configuration.
* **Synchronize** – synchronizes the GUI with the current board configuration.
* **Data** – shows and changes the board control register. It has two tabs, Simple and Detailed.
* **Which channel is controlled in GUI** – selects channel 1 or channel 2. This selection tells the GUI which LMS8001B and SC1905 to control.
* **Message Display** – shows status messages.

The **SC1905** tab opens the SC1905 linearizer control window. The **Modules** menu opens the LMS8001 control window. Both are described in :doc:`setup`.

Data Panel
==========

The Data panel shows the state of the daisy chain register, which is the status and control register of the LimeRFE 8001P board.
Click the **Read** button to read the current register state from the board. The register bits are shown as checked and unchecked fields, and the corresponding hex values are written in the Message Display panel.
Change the fields and click the **Apply** button to write the new configuration to the board.

The Data panel has two tabs, shown in Figure 6 and Figure 7:

* **Simple** – shows a block diagram of one channel of the board. This view is recommended for understanding the main RF blocks of the board.
* **Detailed** – shows all register bits for both channels, and provides more control parameters.

.. tabs::

   .. tab:: Simple

      .. figure:: /images/LimeRFE-8001P_GUI_simple.png
         :width: 600

         Figure 6: LimeRFE 8001P GUI main control window, Data panel – Simple tab active

   .. tab:: Detailed

      .. figure:: /images/LimeRFE-8001P_GUI_detaled.png
         :width: 600

         Figure 7: LimeRFE 8001P GUI main control window, Data panel – Detailed tab active

The correspondence of the fields in the Simple and Detailed tabs is given in Table 2.

.. list-table:: Table 2. Simple and Detailed tabs – field correspondence
   :header-rows: 1

   * - Simple tab
     - Detailed tab
     - Comments
   * - Bypass AMP
     - BP_AMP1 or BP_AMP2
     - Depends on selected channel
   * - Disable AMP
     - SD_AMP1 or SD_AMP2
     - Depends on selected channel
   * - PDA Enable
     - PDA1_EN or PDA2_EN
     - Depends on selected channel
   * - LNA Enable
     - LNA1_EN or LNA2_EN
     - Depends on selected channel
   * - DA Enable
     - DA1_A_EN or DA1_B_EN or DA2_A_EN or DA2_B_EN
     - Depends on selected channel and selected branch (A or B)
   * - A/B (0 – B; 1 – A)
     - DA1_A_B_CTRL or DA2_A_B_CTRL
     - Depends on selected channel
   * - TX/RX (0 – RX; 1 – TX)
     - TXRX_1 or TXRX_2
     - Depends on selected channel
   * - TX Attenuation [dB]
     - TX1 Att. [dB] or TX2 Att. [dB]
     - Depends on selected channel
   * - ORX Attenuation [dB]
     - ORX1 Att. [dB] or ORX2 Att. [dB]
     - Depends on selected channel

The fields of the Detailed tab are described in Table 3.

.. list-table:: Table 3. Data panel, Detailed tab – field description
   :header-rows: 1

   * - Bit name
     - State after reset
     - Bit/field function
   * - SPI_2_MCU_DIR_OUT_in
     - 1
     - Used in 3-wire SPI communication with U18 (ADF4002-RL7).

       Controlled by software, do not change in manual board setup.
   * - LMS8001_#1_SSENn
     - 1
     - Selects U1 (LMS8001B) on the SPI bus.

       Controlled by software, do not change in manual board setup.
   * - LMS8001_#2_SSENn
     - 1
     - Selects U2 (LMS8001B) on the SPI bus.

       Controlled by software, do not change in manual board setup.
   * - EXT_PLL_SSENn
     - 1
     - Selects U18 (ADF4002-RL7) on the SPI bus.

       Controlled by software, do not change in manual board setup.
   * - LMS8001_#1_RESETn
     - 0
     - Resets U1 (LMS8001B).
   * - LMS8001_#2_RESETn
     - 0
     - Resets U2 (LMS8001B).
   * - SC1905_#1_SSENn
     - 1
     - Selects U121 (SC1905) on the SPI bus.

       Controlled by software, do not change in manual board setup.
   * - SC1905_#2_SSENn
     - 1
     - Selects U131 (SC1905) on the SPI bus.

       Controlled by software, do not change in manual board setup.
   * - SC1905_#1_RESETn
     - 0
     - Resets U121 (SC1905).
   * - SC1905_#2_RESETn
     - 0
     - Resets U131 (SC1905).
   * - BP_AMP1
     - 1
     - Bypasses U93, AMP1 (TQL9063).
   * - SD_AMP1
     - 0
     - Shuts down U93, AMP1 (TQL9063).
   * - BP_AMP2
     - 1
     - Bypasses U106, AMP2 (TQL9063).
   * - SD_AMP2
     - 0
     - Shuts down U106, AMP2 (TQL9063).
   * - DA1_A_EN
     - 0
     - Enables U58, DA1_A (MMZ38333).
   * - DA1_B_EN
     - 0
     - Enables U77, DA1_B (QPA9122M).
   * - DA2_A_EN
     - 0
     - Enables U69, DA2_A (MMZ38333).
   * - DA2_B_EN
     - 0
     - Enables U79, DA2_B (QPA9122M).
   * - LNA1_EN
     - 0
     - Enables U46, LNA1 (TSS-53LNB3+).
   * - LNA2_EN
     - 0
     - Enables U48, LNA2 (TSS-53LNB3+).
   * - PDA1_EN
     - 0
     - Enables U101, PDA1 (MNA-7A+).
   * - PDA2_EN
     - 0
     - Enables U103, PDA2 (MNA-7A+).
   * - DA1_A_B_CTRL
     - 0
     - 0 – selects branch B (DA1_B driver amplifier).

       1 – selects branch A (DA1_A driver amplifier).
   * - DA2_A_B_CTRL
     - 0
     - 0 – selects branch B (DA2_B driver amplifier).

       1 – selects branch A (DA2_A driver amplifier).
   * - ORX1 Att. [dB]
     - 31.75
     - Attenuation in the ORX1 path, 0 – 31.75 dB in steps of 0.25 dB.
   * - ORX2 Att. [dB]
     - 31.75
     - Attenuation in the ORX2 path, 0 – 31.75 dB in steps of 0.25 dB.
   * - TX1 Att. [dB]
     - 31.75
     - Attenuation in the TX1 path, 0 – 31.75 dB in steps of 0.25 dB.
   * - TX2 Att. [dB]
     - 31.75
     - Attenuation in the TX2 path, 0 – 31.75 dB in steps of 0.25 dB.
   * - TXRX_1
     - 0
     - 0 – turns on the receiver (LNA1), if the other bits are properly configured.

       1 – turns on the transmitter (PDA1 and DA1_A or DA1_B), if the other bits are properly configured.
   * - TXRX_2
     - 0
     - 0 – turns on the receiver (LNA2), if the other bits are properly configured.

       1 – turns on the transmitter (PDA2 and DA2_A or DA2_B), if the other bits are properly configured.
