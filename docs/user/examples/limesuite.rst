GUI
###

LTE
***

The following example shows the setup of the LimeRFE 8001P board for the following configuration: 

* The board will operate in the following frequency bands:  

    * IF = 2GHz. 
    * RF = 3.5 GHz.  

* Channel CH#1, DA branch A are is used. 
* AMP1 is bypassed. 
* SC1905 linearization is ON and the DA1_A amplifier will be linearized (external coupler will be used). 

Steps for setting up the LimeRFE 8001P board with the configuration given above:

1. Terminate all IF and RF connectors with 50 Ohm terminations, except J6 (IFTX1), J4 (ORX1), J15 (TXRX1)
2. Connect the signal generator to J6 (IFTX1) connector. It is recommended to use attenuators (small attenuation value, for example 2 or 3 dB) between the signal generator and connector J6. Also, for initial test, set the signal power level not higher than –30 dBm. Do not turn on the input signal yet. 
3. External coupler will be connected to the J15 (TXRX1) connector with the other main port to the spectrum analyzer. It is highly recommended to use high power rating, high-level attenuators before the spectrum analyzer, so as to protect it from high power levels. For all our tests a 20 dB / 5 W attenuator was used to protect the spectrum analyzer. The signal from the coupled port of the coupler will be returned to the J4 (ORX1) connector back to the LimeRFE 8001P board. 
4. Connect your PC to the LimeRFE 8001P board through a USB cable. 
5. Place jumpers at positions bottom at JP1 and at position up at JP2. 
6. Connect a bench power supply set to J2 and set 5 VDC and 2 A current limit, then turn on the output. At this point the power supply should show a current consumption of about 230 mA. 
7. Open the LimeRFE 8001P GUI software. 
8. Change communication from SDR to Direct (USB), in the Communication panel. 
9. Refresh the available COM ports by clicking the Refresh button in USB panel. 
10. Select the COM port of the LimeRFE 8001P board. Open the port by clicking the Open button in the Port panel. 
11. Click the Read button in the Data panel and reset state of daisy chain register will be presented in the Data panel fields. 
12. Check the additional fields in the Data panel (LMS8001_#1_RESETn, SC1905_#1_RESETn, DA1_A_EN, PDA1_EN, DA1_A_B_CTRL, TXRX_1). Set TX1 Att. [dB] to 0 dB, in the drop-down menu. No field should be unchecked! 
13. Select “Channel 1” in the drop-down menu of the Which channel is controlled in GUI panel. With this selection, the GUI will know which LMS8001B and SC1905 to use, whether the ones on Channel 1 or on Channel 2. 
14. Click the Apply button in the Data panel to reconfigure the board from its reset state. 
15. Go to Modules and select “LMS8001” to open a new window for the LMS8001 configuration. 
16. elect LMS8001B – High-Linearity Mixer tab, keep CHANNEL C selected and uncheck BIAS_PD and LOBUF_PD fields, in Configuration panel, to turn on a mixer on channel C of LMS8001B chip (Ch.C). Channel C on LMS8001B is used for transmit. 
17. In the mixer of Ch.C (High-Linearity Mixer tab, in the Loss panel, next to Configuration panel) increase the Loss control value to maximum, value of 15. This will insert an additional loss of roughly 9 dB in TX path and prevent PDA and DA to enter in highly saturation region. After initial start of linearization (described below) and confirming that the beginning of the test is successful, this value could be decreased in order to increase DA1_A output power. 
18. n the same window, select CHANNEL D and uncheck BIAS_PD and LOBUF_PD fields to turn on the mixer on channel D of LMS8001B chip (Ch.D). Channel D on LMS8001B is used for observing the signal returned from the amplifier to be linearized.
19. Go to the PLL Profiles tab and in the Smart Tune panel enter 5.5 (to set 5.5 GHz LO frequency) and click the Smart Tune button. 
20. n the LO Distribution panel check the Ch.C and Ch.D fields to deliver the LO signal to mixers in channels C and D. Numbers for 5.5 GHz should be shown in the same panel in the rows for Ch.C and Ch.D indicating successful operation.
21. Set the signal generator to 2 GHz with amplitude of –30 dBm and turn on. This signal should be real LTE signal in order to see, in real time, how SC1905 accomplishes linearization of the Power Amplifier. 
22. The same signal shifted to 3.5 GHz and amplified by about 15 dB should appear on spectrum analyzer. 
23. Now the LMS8001B control window should be minimized and the main control window active. In the main control window change a tab from Main to SC1905 to enter in the SC1905 configuration settings. 
24. In the Status panel change “No refresh” to “0.5 seconds” refresh rate.
25. In the IC configuration panel Product Version field “SC1905” should be written and is an indicator that communication between MCU and SC1905 is established.  
26. Now increase the power of signal generator from –30 dBm to +4 dBm. After this an RFIN Power [dBm] slider in the RF panel should appear (without AMP1 turned ON an optimum signal generator power should be in the range of 0 – 4 dBm to enable normal operation of the SC1905). 
27. Change tab from SC1905 to Main and in the TX Att. [dB] and ORX Att. [dB] fields decrease attenuation slightly (it is suggested is in steps not higher than 2 dB). 
28. After every step, return to the SC1905 tab and check the RFFB Power [dBm] slider. In this step here, Loss control value of 15 could be decreased in order to increase DA1_A output power. This should be done in smaller steps, watching all other parameters of the whole setup. 
29. After every change of TX Att. [dB] and ORX Att. [dB], RFFB signal level will change and the SC1905 will start operation from the beginning in “CAL” mode. 
30. When the RFFB Power [dBm] slider is set at the optimum level (somewhere in the middle) the SC1905 will enter in “CAL” mode once again and will progress through all other modes and finish in “TRACK” mode. 
31. At this point the SC1905 performs linearization and if the signal generator is providing an LTE signal, a decrease of adjacent channel side lobes will be seen. 
32. In order to observe once again how the SC1905 linearizes the Power Amplifier, the SC1905 should be reset in the GUI by pushing the Reset RFPAL button. In this moment a significant increase in side bands should be noticed, but after a while, these side bands will lower levels, indicating a successful SC1905 operation. 