Changelog
#########

The first LimeRFE 8001P was revision v1.0, and so this changelog starts with changes from that point.

v1.1
====

LimeRFE 8001P v1.1 board is designed using the LimeRFE 8001P v1.0 project as base, with the schematic and PCB layout changes described below.
The block diagrams of both versions are shown in Figure 1 and Figure 2.

.. figure:: /images/LimeRFE-8001P_v1.0_BD.png
  :width: 600

  Figure 1: LimeRFE 8001P v1.0 block diagram

.. figure:: /images/LimeRFE-8001P_v1.1_BD.png
  :width: 600

  Figure 2: LimeRFE 8001P v1.1 block diagram

The major changes are:

* Front end section designations changed: PA to DA and DA to PDA.
* LMS8001 channel mapping changed: TX on *Ch.C*, RX on *Ch.B*, ORX on *Ch.D*; *Ch.A* is not used.
* Band-pass filters and fixed attenuators added in the IF section.
* Fixed attenuators and digital step attenuators added in the RF section.
* Microstrip DA output coupler replaced with a discrete coupler.
* REF CLK circuit of channel 2 removed; both channels use one reference clock.
* Level shifters and digital bus switches removed from the linearization section and the LMS8001 GPIO bus.
* I2C connector J1 added.
* SPI1 MISO level shifter U163 added.
* 12 V barrel connector J30 added instead of the 2-pin header J44.

Front End
---------

The front end section designations have been changed:

* PA (power amplifier) to DA (driver amplifier),
* DA (driver amplifier) to PDA (predriver amplifier).

.. note::
   Some of the old names are still used in the GUI (Detailed tab).

.. figure:: /images/LimeRFE-8001P_v1.1_changes_names.png
  :width: 600

  Figure 3: Front end section designations

In v1.0 the total gain of the TX path was larger than optimum, which pushed the PDA and DA into a highly nonlinear region. To reduce the gain, fixed and digital step attenuators have been added to the RF section.

Fixed RF attenuators have been added between the LMS8001 and the PDA: U23, U25 for channel 1 and U20, U22 for channel 2. Figure 4 shows channel 1 only.
These footprints are not fitted by default. Instead, the existing attenuator U107 (U3 for channel 2) has been changed from 3 dB to 6 dB (PAT1220-C-6DB-T5).

.. figure:: /images/LimeRFE-8001P_v1.1_changes_RFatt1.png
  :width: 600

  Figure 4: RF attenuators between LMS8001 and PDA

Fixed RF attenuators have been added in both DA branches: U155, U156, U157, U158 for channel 1 and U159, U160, U161, U162 for channel 2. Figure 5 shows channel 1 only.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_RFatt2.png
  :width: 600

  Figure 5: RF attenuators in DA branches

Digital step attenuators have been added between the PDA and the DA: U71 for channel 1 and U72 for channel 2. Figure 6 shows channel 1 only.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_DSA.png
  :width: 600

  Figure 6: RF digital step attenuator

The microstrip coupler used in v1.0 to sample the DA output has been replaced with the HHM2955A1 discrete coupler, which uses much less board area: U123 for channel 1 and U132 for channel 2. Figure 7 shows channel 1 only.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_coupler.png
  :width: 600

  Figure 7: RF coupler

IF Section
----------

Band-pass filters have been added in the IF section: U133, U137, U136 for channel 1 and U134, U139, U138 for channel 2. Figure 8 shows channel 1 only.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_IFfilters.png
  :width: 600

  Figure 8: IF band-pass filters

The band-pass filter between the SC1905 and the LMS8001 (U63 for channel 1, U66 for channel 2) has been changed from DEA161953BT-2303B1-H to BFCN-1860+, because the 200 MHz pass band of the DEA161953BT-2303B1-H was too narrow for the linearization loop.

Fixed RF attenuators have been added in the IF section, between the SC1905 and the LMS8001: U96, U97, U98 for channel 1 and U99, U150, U151 for channel 2. Figure 9 shows channel 1 only.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_IFatt.png
  :width: 600

  Figure 9: IF attenuators

LMS8001 Channel Mapping
-----------------------

In v1.0 the TX path was routed through channel A (*Ch.A*) of the LMS8001. A 40 MHz clock spur coupled into the *Ch.A* signals, most likely inside the chip, so it could not be avoided by board routing.
The LMS8001 channel mapping has been changed as follows:

* TX part: *Ch.A* to *Ch.C*,
* RX part: *Ch.D* to *Ch.B*,
* ORX part: *Ch.C* to *Ch.D*.

*Ch.B* is not used in v1.0, while *Ch.A* is not used in v1.1.

Linearization
-------------

The level shifters, the digital bus switch and the related connectors in the linearization section have been removed. In v1.0 these were U134, U132, U123, U133, J53, J52 for channel 1 and U136, U137, U138, U139, J55, J51 for channel 2 (v1.0 designators).
The 14-pin SC1905 debug connectors J32 and J48 (v1.0) have also been removed. Figure 10 shows channel 1 only.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_SC1905ctrl.png
  :width: 600

  Figure 10: Removed linearization section control circuits

Clock
-----

The complete REF CLK circuit of channel 2 has been removed. In v1.1 both channels are synchronized to one reference clock.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_REFCLK.png
  :width: 600

  Figure 11: REF CLK circuit

A footprint for the crystal oscillator U152 has been added. U152 is not fitted by default.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_XO.png
  :width: 600

  Figure 12: Crystal oscillator U152

Digital Interfaces
------------------

The I2C connector J1 has been added for front panel communication.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_I2C.png
  :width: 600

  Figure 13: I2C connector J1

The digital bus switches U22 and U23, and the inverter U20, on the LMS8001 GPIO bus have been removed (v1.0 designators).
The LMS8001 GPIO PMOD connectors J30 and J31 and their level shifters U25, U70, U71 and U72 (v1.0 designators) have also been removed.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_GPIOsw.png
  :width: 600

  Figure 14: Removed LMS8001 GPIO bus switches

The level shifter U163 has been added, to drive the SPI1 MISO signal in the reverse direction (from the board to the J28 connector).

.. figure:: /images/LimeRFE-8001P_v1.1_changes_SPI1.png
  :width: 600

  Figure 15: SPI1 MISO level shifter U163

Power Supply
------------

The barrel connector J30 has been placed instead of the 2-pin header J44, for the 12 V DC supply.

.. Note: the source document says "J29". The v1.1 netlist confirms the barrel connector is J30 (RAPC712X);
   J29 is the USB header.

.. figure:: /images/LimeRFE-8001P_v1.1_changes_12V.png
  :width: 600

  Figure 16: 12 V barrel connector J30

v1.0
====

Initial version.
