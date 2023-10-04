# LimeRFE 8001P

![LimeRFE 8001P board](/images/LimeRFE_8001P_2_722w.jpg)

## Contents

The LimeRFE 8001P board is a two channel RF front end module capable of frequency
conversion, RF signal amplification and power amplifier linearisation. It can be
configured so that either the integrated driver amplifiers, or external power
amplifiers, are linearised. LimeRFE 8001P is designed with 5G cellular networks
in mind, but may be put to use in other applications with similar requirements.

The directory structure is as follows:

      firmware/<version>/
          LMS8FE                 - Arduino firmware project
      hardware/<version>/
          BOM/                   - Bill of materials
          Gerbers/               - Gerber manufacturing files
          KiCAD/                 - KiCAD design
          PDF/                   - PDF outputs

Note that the firmware should match the hardware version. E.g. use 1v1 firmware
with 1v1 hardware.

## Licensing

The hardware designs are licensed under the Solderpad Hardware License v2.1.

The firmware is licensed under the Apache License, Version 2.0.
