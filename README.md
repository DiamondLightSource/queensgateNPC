queensgateNPC
=======

EPICS asyn driver for Queensgate NanoScan NPC-D-6xxx and NanoScan NS-D-6xxx controller series compatible with PriorSDK 2.2.16 version and over.
Controllers must have firmware application 6.6.31 or over for full compatibility

Credits and licensing
---------------------

Original development of EPICS source code in this module from Diamond Light Source. Released under the Apache V2 license. See LICENSE.
For the library license please check the one included with the SDK.

Supported platforms
-------------------

This driver has been initially tested on an NPC-D-6330 and an NS-D-6300 controller with RHEL7 
using EPICS BASE R3.14.12.7. Initial version was 1.4.1 of the SDK.

Known issues
------------
Compatibility with NPC-D-5xxx suggested by Prior but not confirmed

SDK
---

Latest SDK/controller interface DLL API can be downloaded from https://www.dropbox.com/s/vb367zzeap6gfr8/controller_interface_2.6.25.zip?dl=0 or by contacting Prior Scientific.
Compatible controller firmware can be downloaded from https://www.dropbox.com/s/so95iss1j1sjl90/npc6000_application.6.6.31.tgz?dl=0

Instructions
------------

To use this driver do the following:

* Download the needed SDK pack.
* Unpack the SDK on `queensgateNPCApp/src/qglib/` and copy your chosen `controller_interface.so` or `controller_interface64.so` file into the `qglib` directory.
* Ensure that the `LIB_INSTALLS+= ...` line points to your chosen `.so` file. Removing the unused `.so` files is recommended in order to reduce the module and IOC size.
* Set paths to dependencies in configure/RELEASE

To update the version of the SDK:

* Replace the SDK on `queensgateNPCApp/src/qglib/` and the `.so` file.
* Ensure that the `LIB_INSTALLS+= ...` line points to your chosen `.so` file.


An example IOC is included at iocs/sampleIOC.

