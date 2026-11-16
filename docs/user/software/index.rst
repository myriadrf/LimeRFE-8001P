Software
########

.. toctree::
   :maxdepth: 0
   :hidden:

   gui
   setup

The LimeRFE 8001P is controlled with the LimeRFE 8001P control software, which has a graphical user interface (GUI).
The GUI is used to configure the RF devices on the board, the LMS8001B frequency shifters and the SC1905 linearizers.

Installing
**********

The control software can be used in two ways, depending on how the board is connected to the host:

* **Direct USB connection** – the standalone LimeRFE 8001P GUI. The software is available for Windows OS.
  The source code is available in the `LimeRFE-8001P_CTL`_ repository.
* **Connection through LimeSDR-PCIe-5G** – the LimeRFE 8001P control window is built into the Lime Suite GUI from the `LimeSuite_5G`_ repository (branch **master_branch_update**).
  For installation of this Lime Suite GUI version, please refer to the `LimeSuite_5G documentation`_.

.. TODO: add installation / build instructions for the standalone GUI from the LimeRFE-8001P_CTL repository
   (prebuilt binary or build steps, drivers needed for the USB COM port).

.. warning::
   LimeRFE 8001P is not supported by the standard :external+suite:ref:`Lime Suite <index:introduction>` or :external+suiteng:ref:`Lime Suite NG <index:introduction>` releases.
   Use the standalone GUI or the LimeSuite_5G version of Lime Suite GUI.

The control windows are the same in both cases. The GUI is described in :doc:`gui`, and the connection and configuration procedure in :doc:`setup`.

.. note::
   In LimeRFE 8001P v1.1 the front end amplifiers were renamed: PA (power amplifier) to DA (driver amplifier), and DA (driver amplifier) to PDA (predriver amplifier).
   Some of the old names are still used in the GUI (Detailed tab of the Data panel).

.. _LimeRFE-8001P_CTL: https://github.com/myriadrf/LimeRFE-8001P_CTL
.. _LimeSuite_5G: https://gitlab.com/myriadrf/LimeSuite_5G
.. _LimeSuite_5G documentation: https://limeadpd.myriadrf.org/limesuite/index.html
