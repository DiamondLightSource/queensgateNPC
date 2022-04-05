TOP=../..

include $(TOP)/configure/CONFIG

# ------------------------------
# Build a Diamond Support Module
# ------------------------------
LIBRARY_IOC += queensgateNPC

queensgateNPC_DIR = $(TOP)/queensgateNPCApp/src

LIB_INSTALLS += ../qglib/controller_interface/bin/linux/controller_interface64.so

# DIRS += adapter/source
# DIRS += adapter

# include files
# INCLUDE_DIRS += adapter/include/
USR_CPPFLAGS += -I../adapter/include/
USR_CPPFLAGS += -I../include/

# xxxRecord.dbd will be installed into <top>/dbd
DBD += queensgateNPC.dbd
DBD += systemCommandSupport.dbd

queensgateNPC_DBD += base.dbd
queensgateNPC_DBD += asyn.dbd
queensgateNPC_DBD += motorSupport.dbd
queensgateNPC_DBD += systemCommandSupport.dbd
queensgateNPC_DBD += queensgateNPC.dbd

# The following are compiled and added to the support library
queensgateNPC_SRCS += TakeLock.cpp
queensgateNPC_SRCS += FreeLock.cpp

#NOTE: Please manually copy dll_adapter.cpp from src/adapter/source/ to src/
queensgateNPC_SRCS += dll_adapter.cpp
queensgateNPC_SRCS += queensgateNPCcontroller.cpp
queensgateNPC_SRCS += queensgateNPCaxis.cpp
queensgateNPC_SRCS += queensgateNPCregistrar.cpp

# We need to link against the EPICS Base libraries
queensgateNPC_LIBS += motor
queensgateNPC_LIBS += asyn
queensgateNPC_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

$(info end of MAKEFILE) 

