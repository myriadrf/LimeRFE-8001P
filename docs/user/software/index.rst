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
==========

The LimeRFE 8001P GUI is a standalone application, which controls the board through the USB connection.
The source code is available in the `LimeRFE-8001P_CTL`_ repository, and the GUI is built from it.

.. note::
   Work is also underway to integrate support for the LimeRFE 8001P into the :external+suiteng:ref:`Lime Suite NG <index:introduction>` software.

Building on Linux
-----------------

The procedure below was tested on Ubuntu 24.04.

#. Install the build tools and the wxWidgets development package:

   .. code-block:: bash

      sudo apt install build-essential cmake git libwxgtk3.2-dev

#. Download the source code:

   .. code-block:: bash

      git clone https://github.com/myriadrf/LimeRFE-8001P_CTL.git
      cd LimeRFE-8001P_CTL

#. Build the software:

   .. code-block:: bash

      mkdir -p build
      cd build
      cmake ..
      make -j$(nproc)

   The GUI, together with the command line tools (LimeRFE_8001P_start and LimeRFE_8001P_stop) and the examples, is built in the ``build/bin`` folder.

#. Allow your user to access the USB serial port of the board. On Linux the board appears as a ``/dev/ttyACM*`` device, which is accessible to the members of the ``dialout`` group:

   .. code-block:: bash

      sudo usermod -aG dialout $USER

   Log out and log in again for the change to take effect.

#. Start the GUI:

   .. code-block:: bash

      ./bin/LimeRFE_8001P

The GUI is described in :doc:`gui`, and the connection and configuration procedure in :doc:`setup`.

.. _LimeRFE-8001P_CTL: https://github.com/myriadrf/LimeRFE-8001P_CTL
