TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += queensgateNPC

queensgateNPC_DIR = $(TOP)/queensgate_NPCApp/src
LIB_INSTALLS += ../controller_interface64.so

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h

# xxxRecord.dbd will be installed into <top>/dbd
DBD += queensgateNPC.dbd
DBD += systemCommandSupport.dbd

queensgateNPC_DBD += base.dbd
queensgateNPC_DBD += asyn.dbd
queensgateNPC_DBD += motorSupport.dbd
#queensgateNPC_DBD += genSubRecord.dbd
queensgateNPC_DBD += systemCommandSupport.dbd
queensgateNPC_DBD += queensgateNPC.dbd

# The following are compiled and added to the support library
queensgateNPC_SRCS += TakeLock.cpp
queensgateNPC_SRCS += FreeLock.cpp
queensgateNPC_SRCS += dll_adapter.cpp
queensgateNPC_SRCS += queensgateNPCcontroller.cpp
queensgateNPC_SRCS += queensgateNPCaxis.cpp
queensgateNPC_SRCS += queensgateNPCregistrar.cpp

# We need to link against the EPICS Base libraries
queensgateNPC_LIBS += motor
queensgateNPC_LIBS += asyn
queensgateNPC_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
