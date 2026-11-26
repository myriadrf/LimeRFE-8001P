Setup
#####

The LimeRFE 8001P basic setup procedure consists of four steps:

#. Establishing the connection with the board.
#. Initial RF configuration – configuration of all RF function blocks, except the LMS8001B and SC1905.
#. Configuration and setup of the LMS8001B.
#. Configuration and setup of the SC1905.

Before starting, connect the power supply and set the JP1 and JP2 jumpers, see :doc:`/user/setup`.

Establishing Connection
=======================

The LimeRFE 8001P board is connected to the host PC through the micro USB connector (CON1):

#. Connect the LimeRFE 8001P micro USB connector (CON1) to the host PC.
#. Open the LimeRFE 8001P GUI. The main control window appears (Figure 5 in :doc:`gui`). Check **Direct (USB)** in the Communication panel.
#. Click the **Refresh** button in the USB panel. All available serial ports (``/dev/ttyACM*``) appear in the drop-down menu.
#. Select the serial port of the LimeRFE 8001P board (usually ``/dev/ttyACM0``) and click the **Open** button in the Port panel.
#. Three lines of short messages appear in the Message Display panel, confirming a successful connection.

Initial RF Configuration
========================

After the connection is established, the RF devices on the board (amplifiers, switches and attenuators) are configured in the Data panel:

#. Click the **Read** button in the Data panel. The current state of the board control register is shown (see :doc:`gui`).
#. Open the Detailed tab and set the fields as given in Table 4, for the selected channel, mode and driver amplifier branch.
#. Click the **Apply** button to write the configuration to the board.

In TX mode, pay attention to the TX1 Att. [dB] and TX2 Att. [dB] values. After reset both are set to the maximum attenuation, 31.75 dB.

.. table:: Table 4. Recommended values of the initial RF configuration (Data panel, Detailed tab)

   +---------+----------+------------+----------------------+----------------------+
   | **Ch.** | **Mode** | **Branch** | **Main bits**        | **Additional bits**  |
   +=========+==========+============+======================+======================+
   | #1      | TX       | A          | DA1_A_EN = 1         | DA1_B_EN = 0         |
   |         |          |            |                      |                      |
   |         |          |            | PDA1_EN = 1          | LNA1_EN = 1          |
   |         |          |            |                      |                      |
   |         |          |            | DA1_A_B_CTRL = 1     | SD_AMP1 = 0          |
   |         |          |            |                      |                      |
   |         |          |            | TXRX_1 = 1           | BP_AMP1 = X          |
   |         |          +------------+----------------------+----------------------+
   |         |          | B          | DA1_B_EN = 1         | DA1_A_EN = 0         |
   |         |          |            |                      |                      |
   |         |          |            | PDA1_EN = 1          | LNA1_EN = 1          |
   |         |          |            |                      |                      |
   |         |          |            | DA1_A_B_CTRL = 0     | SD_AMP1 = 0          |
   |         |          |            |                      |                      |
   |         |          |            | TXRX_1 = 1           | BP_AMP1 = X          |
   |         +----------+------------+----------------------+----------------------+
   |         | RX       | A          | LNA1_EN = 1          | DA1_A_EN = 1         |
   |         |          |            |                      |                      |
   |         |          |            | TXRX_1 = 0           | DA1_B_EN = 0         |
   |         |          |            |                      |                      |
   |         |          |            |                      | PDA1_EN = 1          |
   |         |          |            |                      |                      |
   |         |          |            |                      | DA1_A_B_CTRL = 1     |
   |         |          |            |                      |                      |
   |         |          |            |                      | SD_AMP1 = 0          |
   |         |          |            |                      |                      |
   |         |          |            |                      | BP_AMP1 = X          |
   |         |          +------------+----------------------+----------------------+
   |         |          | B          | LNA1_EN = 1          | DA1_A_EN = 0         |
   |         |          |            |                      |                      |
   |         |          |            | TXRX_1 = 0           | DA1_B_EN = 1         |
   |         |          |            |                      |                      |
   |         |          |            |                      | PDA1_EN = 1          |
   |         |          |            |                      |                      |
   |         |          |            |                      | DA1_A_B_CTRL = 0     |
   |         |          |            |                      |                      |
   |         |          |            |                      | SD_AMP1 = 0          |
   |         |          |            |                      |                      |
   |         |          |            |                      | BP_AMP1 = X          |
   +---------+----------+------------+----------------------+----------------------+
   | #2      | TX       | A          | DA2_A_EN = 1         | DA2_B_EN = 0         |
   |         |          |            |                      |                      |
   |         |          |            | PDA2_EN = 1          | LNA2_EN = 1          |
   |         |          |            |                      |                      |
   |         |          |            | DA2_A_B_CTRL = 1     | SD_AMP2 = 0          |
   |         |          |            |                      |                      |
   |         |          |            | TXRX_2 = 1           | BP_AMP2 = X          |
   |         |          +------------+----------------------+----------------------+
   |         |          | B          | DA2_B_EN = 1         | DA2_A_EN = 0         |
   |         |          |            |                      |                      |
   |         |          |            | PDA2_EN = 1          | LNA2_EN = 1          |
   |         |          |            |                      |                      |
   |         |          |            | DA2_A_B_CTRL = 0     | SD_AMP2 = 0          |
   |         |          |            |                      |                      |
   |         |          |            | TXRX_2 = 1           | BP_AMP2 = X          |
   |         +----------+------------+----------------------+----------------------+
   |         | RX       | A          | LNA2_EN = 1          | DA2_A_EN = 1         |
   |         |          |            |                      |                      |
   |         |          |            | TXRX_2 = 0           | DA2_B_EN = 0         |
   |         |          |            |                      |                      |
   |         |          |            |                      | PDA2_EN = 1          |
   |         |          |            |                      |                      |
   |         |          |            |                      | DA2_A_B_CTRL = 1     |
   |         |          |            |                      |                      |
   |         |          |            |                      | SD_AMP2 = 0          |
   |         |          |            |                      |                      |
   |         |          |            |                      | BP_AMP2 = X          |
   |         |          +------------+----------------------+----------------------+
   |         |          | B          | LNA2_EN = 1          | DA2_A_EN = 0         |
   |         |          |            |                      |                      |
   |         |          |            | TXRX_2 = 0           | DA2_B_EN = 1         |
   |         |          |            |                      |                      |
   |         |          |            |                      | PDA2_EN = 1          |
   |         |          |            |                      |                      |
   |         |          |            |                      | DA2_A_B_CTRL = 0     |
   |         |          |            |                      |                      |
   |         |          |            |                      | SD_AMP2 = 0          |
   |         |          |            |                      |                      |
   |         |          |            |                      | BP_AMP2 = X          |
   +---------+----------+------------+----------------------+----------------------+

LMS8001B
========

After the initial RF configuration, configure and start the LMS8001B:

#. Select the channel (Channel 1 or Channel 2) in the **Which channel is controlled in GUI** panel.
#. In the Detailed tab of the Data panel, check LMS8001_#1_RESETn for channel 1, or LMS8001_#2_RESETn for channel 2, and click **Apply**. This takes the LMS8001B out of its reset state.
#. Open **Modules** |rarr| **LMS8001**. A new window appears, where the LMS8001B is configured.
#. Open the **LMS8001B - High-Linearity Mixer** tab, shown in Figure 8, and select the channel: CHANNEL B, C or D.

   .. note::
      Channel C is used for TX, channel D for ORX and channel B for RX. Channel A is not used in normal operation of the LimeRFE 8001P.

#. Turn on the mixer of the selected channel by unchecking the BIAS_PD and LOBUF_PD fields in the Configuration panel.
#. Open the **PLL Profiles** tab, shown in Figure 9, and enter the LO frequency.
#. Click the **Smart Tune** button to tune the PLL.
#. If PLL tuning is successful, confirmation messages appear in the Message Display panel and the Active Profile Status shows Lock (left side of the window in Figure 9).
#. Deliver the LO signal to the mixers by checking the *Ch.B*, *Ch.C* and/or *Ch.D* fields in the LO Distribution panel.

The basic LMS8001B setup is now complete.

.. figure:: /images/LimeRFE-8001P_GUI_LMS8001P.png
  :width: 600

  Figure 8: LMS8001 control window – LMS8001B - High-Linearity Mixer tab

.. figure:: /images/LimeRFE-8001P_GUI_PLL.png
  :width: 600

  Figure 9: LMS8001 control window – PLL Profiles tab

SC1905
======

The final step is the setup and start of the SC1905:

#. Select the channel (Channel 1 or Channel 2) in the **Which channel is controlled in GUI** panel of the main control window.
#. In the Detailed tab of the Data panel, check SC1905_#1_RESETn or SC1905_#2_RESETn, depending on the selected channel, and click **Apply**. This takes the SC1905 out of its reset state.
#. Open the **SC1905** tab. The SC1905 control window appears, as shown in Figure 10.
   This part of the GUI is based on the original Maxim GUI for the SC1905 evaluation boards, and has most of its functions.
#. Click the **Force Update** button. "SC1905" must appear in the Product Version field, and the FW Version and Frequency Range fields in the IC Configuration panel should be the same as, or similar to, Figure 10.
   This confirms the communication between the MCU and the SC1905.
#. The Frequency Range field in the Configure Application panel must be the same as in Figure 10.
   For finer frequency scanning, enter the Min Frequency and Max Frequency values. It is recommended to use 2.5 x the signal bandwidth below and above the IF carrier frequency.
   For example, for a 2 GHz IF carrier and 20 MHz signal bandwidth, the values are 1950 and 2050.
#. The SC1905 linearizer is now configured and started.
#. For best operation, the signals at the SC1905 RFIN and RFFB inputs must have the correct power level:

   * RFIN – in tests with LTE 4G and 5G signals (PAR = 8.5 dB, 20 MHz bandwidth), the best operation was achieved with an average RFIN input power of 0 – 4 dBm. At this power the RFIN Power [dBm] slider is close to the middle.
   * RFFB – set the RFFB Power [dBm] slider to its optimal position by adjusting TX1 Att. [dB] and ORX1 Att. [dB] (or TX2 Att. [dB] and ORX2 Att. [dB]) in the main control window.

   .. warning::
      Avoid too high power at the SC1905 inputs. The maximum allowed RMS power before the balun at the RFFB input is 7 dBm.

#. If the signal power levels are correct, the SC1905 goes through its modes of operation ("INIT", "CAL", "PDET", "FSA" and "TRACK"), shown in the Overall Status field.
   In "TRACK" mode the SC1905 has completed its setup and coefficient optimization, and amplifier linearization is running.
#. To observe the operation, change the Update Rate from "No refresh" to "0.5 seconds" or "1 second".

.. figure:: /images/LimeRFE-8001P_GUI_SC1905.png
  :width: 600

  Figure 10: SC1905 control window

For a complete setup example, see :doc:`/user/examples/index`.

