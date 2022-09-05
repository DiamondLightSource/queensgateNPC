#include <stdlib.h>

#include <iocsh.h>
#include <epicsExport.h>
#include <asynOctetSyncIO.h>

#include "queensgateNPCcontroller.hpp"
#include "queensgateNPCaxis.hpp"

/* The following functions have C linkage and can be called directly or from iocsh */
extern "C" {

/** Create a controller
 * \param[in] ctrlName Asyn port name
 * \param[in] lowlevelPortAddress Address of the physical port (usually the IP address for Ethernet or/dev/ttyX for serial)
 * \param[in] numAxes Number of configured axes
 * \param[in] movingPollPeriod The period at which to poll position while moving, in seconds
 * \param[in] idlePollPeriod The period at which to poll position while not moving, in seconds
 * \param[in] libPath Full file name and path to the Queensgate controller library
 */
asynStatus qgateControllerConfig(const char* ctrlName, 
                                const char* lowlevelPortAddress,
                                const int maxNumAxes,
                                const double movingPollPeriod, 
                                const double idlePollPeriod, 
                                const char* libPath) {
    //NOTE: For using serial comms to Ethernet Terminal Servers, the Queensgate SDK library might need to 
    // run first on the IOC server a socat connection to link serial comms to an Ethernet Terminal Server. 
    // For example, a /tmp/vmodem0 connecting to port 17 of terminal server 172.23.112.6:
    //      socat PTY,link=/tmp/vmodem0,raw TCP4:172.23.112.6:4017
    //  and then call this code in the IOC:
    //      t = qg.OpenSession("/tmp/vmodem0")
      
    new QgateController(ctrlName, lowlevelPortAddress, maxNumAxes, 
                        movingPollPeriod, idlePollPeriod, libPath);

    return asynSuccess;
}

/** Create an axis
 * \param[in] ctlrName Asyn port name of the controller
 * \param[in] axisNum The number of this axis
 * \param[in] axisNum Name assigned to this axis
 * \param[in] axisType Type of stage attached: motion stage or sensor
 * \param[in] axisMode Mode or confirming when the stage is in position: Native, Unconfirmed, Window-confirmed, LPF-confirmed, Window&LPF
 */
asynStatus qgateAxisConfig(const char* ctrlName, 
                            unsigned int axisNum, 
                            const char* axisName, 
                            unsigned int axisMode=QgateAxis::AXISMODE_NATIVE,
                            unsigned int axisType=QgateAxis::AXISTYPE_STAGE) {
    asynStatus result = asynSuccess;

    //Find controller
    QgateController* ctrl = (QgateController*)findAsynPortDriver(ctrlName);
    if(ctrl == NULL) {
        printf("queensgateNPC: Axis %s could not find NPC controller object '%s'\n", 
                axisName, ctrlName);
        result = asynError;
    } else {
        new QgateAxis(*ctrl, axisNum, axisName, axisMode, axisType);
    }
    return result;
}

} /* end extern "C" */

static const iocshArg qgateCtrlConfig_Arg0 = { "name", iocshArgString };
static const iocshArg qgateCtrlConfig_Arg1 = { "portAddress", iocshArgString };
static const iocshArg qgateCtrlConfig_Arg2 = { "numAxes", iocshArgInt };
static const iocshArg qgateCtrlConfig_Arg3 = { "fastPollPeriodsec", iocshArgDouble };
static const iocshArg qgateCtrlConfig_Arg4 = { "slowPollPeriodsec", iocshArgDouble };
static const iocshArg qgateCtrlConfig_Arg5 = { "libPath", iocshArgString };
static const iocshArg * const qgateCtrlConfig_Args[] = { &qgateCtrlConfig_Arg0, 
                                                        &qgateCtrlConfig_Arg1, 
                                                        &qgateCtrlConfig_Arg2, 
                                                        &qgateCtrlConfig_Arg3, 
                                                        &qgateCtrlConfig_Arg4, 
                                                        &qgateCtrlConfig_Arg5 
                                                        };
static const iocshFuncDef qgateCtrlConfig_FuncDef = { "qgateCtrlConfig", 6, qgateCtrlConfig_Args };

static void qgateCtrlConfig_CallFunc(const iocshArgBuf *args) {
    qgateControllerConfig(args[0].sval, args[1].sval, args[2].ival, 
                            args[3].dval, args[4].dval, args[5].sval);
}

static const iocshArg qgateAxisConfig_Arg0 = { "controller port name", iocshArgString };
static const iocshArg qgateAxisConfig_Arg1 = { "axis index number", iocshArgInt };
static const iocshArg qgateAxisConfig_Arg2 = { "axis name", iocshArgString };
static const iocshArg qgateAxisConfig_Arg3 = { "axis position mode", iocshArgInt };
static const iocshArg qgateAxisConfig_Arg4 = { "axis type", iocshArgInt };
static const iocshArg * const qgateAxisConfig_Args[] = { &qgateAxisConfig_Arg0, 
                                                        &qgateAxisConfig_Arg1, 
                                                        &qgateAxisConfig_Arg2,
                                                        &qgateAxisConfig_Arg3,
                                                        &qgateAxisConfig_Arg4 };
static const iocshFuncDef qgateAxisConfig_FuncDef = { "qgateAxisConfig", 5, qgateAxisConfig_Args };

static void qgateAxisConfig_CallFunc(const iocshArgBuf *args) {
    qgateAxisConfig(args[0].sval, args[1].ival, args[2].sval, 
                        args[3].ival, args[4].ival);
}

/* Export the interface function table to EPICS */
static void npcRegistrar(void)
{
    iocshRegister(&qgateCtrlConfig_FuncDef, qgateCtrlConfig_CallFunc);
    iocshRegister(&qgateAxisConfig_FuncDef, qgateAxisConfig_CallFunc);
}
epicsExportRegistrar(npcRegistrar);


