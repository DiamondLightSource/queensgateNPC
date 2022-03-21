queensgateNPC
=======

EPICS asyn driver for Queensgate NPC-D-5xxx and NPC-D-6xxx series compatible with PriorSDK 2.2 version and over.

Credits and licensing
---------------------

Original development of source code in this module from Diamond Light Source. Released under 
the Apache V2 license. See LICENSE.

Supported platforms
-------------------

This driver has been initially tested on an NPC-D-6330 controller with RHEL7 
using EPICS BASE R3.14.12.7. Initial version was 1.4.1 of the SDK.

Known issues
------------
Compatibility with NPC-D-5xxx suggested by Prior but not confirmed

SDK
---

If you want to change the version of the SDK:

* Replace include files in queensgateNPCApp/src/include
* Replace controller_interface64.so in queensgateNPCApp/src

Instructions
------------

To use this driver do the following:

* Set paths to dependencies in configure/RELEASE

An example IOC is included at iocs/sampleIOC.

