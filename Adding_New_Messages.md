## How to add new messages to the SparkFun u-blox GNSS Arduino Library

Based on [this issue](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/issues/97), here is a summary of how to add new messages to the SparkFun u-blox GNSS Arduino Library with full "auto" support (for callbacks, logging, etc.).

Looking at the issue, we see that the library is not supporting the UBX-NAV-PVAT (Navigation Position Velocity Attitude Time solution).
PVAT is a new message added in version 1.21 of the HPS (High Precision Fusion) firmware and version 33.21 of the F9 Interface Description.
This makes us wonder if more new messages have been added which should also be included?

### Step 1: Check the Interface Description for new keys

* Download the latest [interface description](https://www.u-blox.com/sites/default/files/F9-HPS-1.21_InterfaceDescription_UBX-21019746.pdf) from the [u-blox website](https://www.u-blox.com/en/product/zed-f9r-module#tab-documentation-resources)
* Open the interface description in Adobe Acrobat Reader DC (the free version)
* Do a ```File \ Save as Text...```
* Save the file in ```Text (Accessible) (*.txt)``` format
* Go make a cup of tea - this takes a while
* Open the txt file in Notepad++ or another editor which supports Regular Expressions
* The keys will have been saved as individual lines in the format: 0xnnnnnnnn space CR LF
* So all we need to do is use a regex to delete everything else
* Open Search \ Replace
* Click the Search Mode - Regular Expression button
* In the "Find what :" box enter: ```^(?!.*0x[\dabcdefABCDEF]{8}\s\r\n).*```
* Clear the "Replace with :" box
* Click "Replace All"
* You are left with just the keys - and a bunch of empty lines, some of which contain form feeds (\f)
* Delete the empty lines (\r\n) by replacing \r\n with nothing - don't panic, this takes a few seconds
* Delete the form feeds by replacing \f with nothing
* Finally replace the remaining spaces (\s) with \r\n
* Delete any spurious lines left at the start of the file. E.g. ROM and BASE and 0x118B2060. These came from the General information section
* The following line (0x10340014) is the first key from the "Configuration Reference" section
* Search for that key number and you will find it again half way through the file. This second copy came from "Configuration Defaults"
* Delete the duplicate keys from that line onwards
* Save the file
* Open it in a spreadsheet, e.g. LibreOffice Calc
* Select the "A" column and click "Sort Ascending A-Z"
* Save the file (as Text CSV)
* Use KDiff3 or another diff package to see the new additions

You can find the keys in the [keys folder](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/tree/main/keys), saved as sorted text files.
There are separate files for the P, R and T interfaces, plus a combined list (also sorted in ascending order).

Comparing HPS 1.21 to HPS 1.20, we can see that the following keys have been added:

* 0x10340014 CFG-BDS-USE_GEO_PRN
* 0x10710005 CFG-I2CINPROT-SPARTN
* 0x10730005 CFG-UART1INPROT-SPARTN
* 0x10750005 CFG-UART2INPROT-SPARTN
* 0x10770005 CFG-USBINPROT-SPARTN
* 0x10790005 CFG-SPIINPROT-SPARTN
* 0x20050035 CFG-TP-DRSTR_TP1
* 0x20910605 CFG-MSGOUT-UBX_RXM_SPARTN_I2C
* 0x20910606 CFG-MSGOUT-UBX_RXM_SPARTN_UART1
* 0x20910607 CFG-MSGOUT-UBX_RXM_SPARTN_UART2
* 0x20910608 CFG-MSGOUT-UBX_RXM_SPARTN_USB
* 0x20910609 CFG-MSGOUT-UBX_RXM_SPARTN_SPI
* 0x2091062a CFG-MSGOUT-UBX_NAV_PVAT_I2C
* 0x2091062b CFG-MSGOUT-UBX_NAV_PVAT_UART1
* 0x2091062c CFG-MSGOUT-UBX_NAV_PVAT_UART2
* 0x2091062d CFG-MSGOUT-UBX_NAV_PVAT_USB
* 0x2091062e CFG-MSGOUT-UBX_NAV_PVAT_SPI
* 0x20910634 CFG-MSGOUT-UBX_SEC_SIG_I2C
* 0x20910635 CFG-MSGOUT-UBX_SEC_SIG_UART1
* 0x20910636 CFG-MSGOUT-UBX_SEC_SIG_UART2
* 0x20910637 CFG-MSGOUT-UBX_SEC_SIG_USB
* 0x20910638 CFG-MSGOUT-UBX_SEC_SIG_SPI

Interestingly, we can also see that one key has been deleted:

* 0x10530006 CFG-UART2-REMAP

From this we can confirm - as documented by u-blox in the [Release Notes](https://www.u-blox.com/sites/default/files/ZED-F9R-02B_FW1.00HPS1.21_RN_UBX-21035491_1.3.pdf) -
that HPS 1.21:

* adds support for SPARTN (Safe Position Augmentation for Real-Time Navigation) correction messages
* enables the use of BeiDou geostationary satellites (previously, this configuration item had a different name)
* enables UBX-SEC-SIG message (signal security measures) as output across the different interfaces
* enables UBX_NAV_PVAT message (navigation and altitude position) as output across the different interfaces

There are also two new dynamic models, robotic lawn mower (11) and e-scooter model (12), which we need to add to the library.

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/805aab18b6656513bfee473487a437754cd3965d) for the changes.

### Step 2: Update the combined keys file

Update [u-blox_config_keys_sorted.txt](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/blob/main/keys/u-blox_config_keys_sorted.txt)
to include the new keys.

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/8895764f237ae494dcd0fa1ae942d487d2e1557f) for the changes.

### Step 3: Update u-blox_config_keys.h

Update [u-blox_config_keys.h](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/blob/main/src/u-blox_config_keys.h) to include the new keys.
Include the descriptions as defined in the Interface Description.

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/3609da15f90a7a66b41524e77c6dc3dd76cd362c) for the changes.

### Step 4: Add the new message struct to u-blox_struct.h

The next step is to add the new struct for UBX-NAV-PVAT to u-blox_struct.h.

The messages are in ascending class and ID order. So we add UBX-NAV-PVAT (0x01 0x17) after UBX-NAV-HPPOSLLH (0x01 0x14).

The names and widths of the fields are taken directly from the interface definition.

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/a4ba440c6240e0974c27f40b976a5ddf0fbdb9b6) for the changes.

### Step 5: Update SparkFun_u-blox_GNSS_Arduino_Library.h

Add the new message ID: ```const uint8_t UBX_NAV_PVAT = 0x17;```

Add the new functions to provide "auto" support for UBX-NAV-PVAT: ```getNAVPVAT```, ```setAutoNAVPVAT```, ..., ```logNAVPVAT```

Add new helper functions to access the most important fields: ```getVehicleRoll```, ..., ```getMotionHeading```

Add the pointer to the struct storage: ```UBX_NAV_PVAT_t *packetUBXNAVPVAT = NULL;```

Add the private init function: ```bool initPacketUBXNAVPVAT();```

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/423a1e2ccd418dd679257edc6edeec0bd3029052) for the changes.

### Step 6: Update SparkFun_u-blox_GNSS_Arduino_Library.cpp

Now we need to update SparkFun_u-blox_GNSS_Arduino_Library.cpp:

#### Step 6.1: Update end()

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/35d225e3f1abb316eda3becb7f8e2eb04ff1d17c) for the changes.

#### Step 6.2: Update checkAutomatic()

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/b746d8e2742961ede95e2d06d5db3a3a557e571d) for the changes.

#### Step 6.3: Update getMaxPayloadSize()

#### Step 6.4: Update processUBXpacket()

Take time to double-check that you have used the correct data width, signed/unsigned and position for each field.

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/8eecdd5044f810b0e2b567150ff63a17c219fe8e) for the changes.

#### Step 6.5: Update checkCallbacks()

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/b53bffaa3ae12482cfb268f23796963d0b8519c9) for the changes.

#### Step 6.6: Add the "auto" functions

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/e394ae003ad38117d150598774d0552059416473) for the changes.

#### Step 6.7: Add the helper functions (if any)

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/318e76383e96d6676bbb57294c25e665c0d4a31f) for the changes.

### Step 7: Add an example

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/06014dc95f1b9ffae4876fbacfb9390541d7c31d) for the changes.

### Step 8: Update keywords.txt

Add the new "auto" and helper functions to keywords.txt.

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/4f0a0ca3c5e6420be9064b91702947c23104bd1b) for the changes.

### Step 9: Update Theory.md

Add the new message to the list of "auto" messages.

See [this commit](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/commit/57f133259245d8071c73797e4be2ff630c2720ab) for the changes.

That's all folks!