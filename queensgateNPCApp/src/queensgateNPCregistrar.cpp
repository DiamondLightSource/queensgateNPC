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
 * \param[in] lowlevelPortAddress Address of the physical port (usually /dev/ttyX)
 * \param[in] numAxes Maximum number of configured axes
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
    //For Queensgate SDK library, it might need to run first (on e.g. /tmp/vmodem0):
    //      socat PTY,link=/tmp/vmodem0,raw TCP4:172.23.112.6:4017
    //  and then call in this code:
    //      t = qg.OpenSession("/tmp/vmodem0")
      
    //printf("New controller(%s, maxNumAxes=%d, lowPort=%s)...\n",ctrlName, maxNumAxes, lowlevelPortAddress);

    //XXX: remove pd, or leave for asynPrint?
    // QgateController* pd = 

    new QgateController(ctrlName, lowlevelPortAddress, maxNumAxes, 
                        movingPollPeriod, idlePollPeriod, libPath);

    // printf("queensgateNPC: Created %p\n", pd);
    return asynSuccess;
}

/** Create an axis
 * param[in] ctlrName Asyn port name of the controller
 * param[in] axisNum The number of this axis
 * param[in] axisNum Name assigned to this axis
 */
asynStatus qgateAxisConfig(const char* ctrlName, unsigned int axisNum, const char* axisName, unsigned char axisType=0) {
    asynStatus result = asynSuccess;

    //XXX:
    // printf("about to create Axis obj for %s controller\n", ctrlName);

    //Find controller
    QgateController* ctrl = (QgateController*)findAsynPortDriver(ctrlName);
    if(ctrl == NULL)
    {
        printf("queensgateNPC: Axis %s could not find NPC controller object '%s'\n", 
                axisName, ctrlName);
        result = asynError;
    }
    else
    {
        //XXX:
        // printf("For axis %d found one!: %p\n", axisNum, ctrl);

        new QgateAxis(*ctrl, axisNum, axisName, axisType);

        //TODO: convert these printfs into asynPrint
        printf("queensgateNPC: %s:Axis %d '%s' %d created\n", ctrlName, axisNum, axisName, axisType);
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
    // XXX: print parameters
    // printf("Controller CallFunc Params:\n");
    // for(int i=0; i<=5; i++)
    // {
    //     switch(i) {
    //         case 2:
    //             printf("\t[%d]='%d'\n", i, args[i].ival);
    //             break;
    //         case 3:
    //         case 4:
    //             printf("\t[%d]='%f'\n", i, args[i].dval);
    //             break;
    //         default:
    //             printf("\t[%d]='%s'\n", i, args[i].sval);
    //             break;
    //     }
    // }
    
    qgateControllerConfig(args[0].sval, args[1].sval, args[2].ival, 
                            args[3].dval, args[4].dval, args[5].sval);
}

static const iocshArg qgateAxisConfig_Arg0 = { "controller port name", iocshArgString };
static const iocshArg qgateAxisConfig_Arg1 = { "axis index number", iocshArgInt };
static const iocshArg qgateAxisConfig_Arg2 = { "axis name", iocshArgString };
static const iocshArg qgateAxisConfig_Arg3 = { "axis type", iocshArgInt };
static const iocshArg * const qgateAxisConfig_Args[] = { &qgateAxisConfig_Arg0, 
                                                        &qgateAxisConfig_Arg1, 
                                                        &qgateAxisConfig_Arg2,
                                                        &qgateAxisConfig_Arg3 };
static const iocshFuncDef qgateAxisConfig_FuncDef = { "qgateAxisConfig", 4, qgateAxisConfig_Args };

static void qgateAxisConfig_CallFunc(const iocshArgBuf *args) {
    // XXX: print parameters
    // printf("Axis CallFunc Params:\n");
    // for(int i=0; i<=2; i++)
    // {
    //     if(i==1)
    //         printf("\t[%d]='%d'\n", i, args[i].ival);
    //     else
    //     {
    //         printf("\t[%d]='%s'\n", i, args[i].sval);
    //     }
    // }
            
    qgateAxisConfig(args[0].sval, args[1].ival, args[2].sval, args[3].ival);
}

/* Export the interface function table to EPICS */
static void npcRegistrar(void)
{
    iocshRegister(&qgateCtrlConfig_FuncDef, qgateCtrlConfig_CallFunc);
    iocshRegister(&qgateAxisConfig_FuncDef, qgateAxisConfig_CallFunc);
}
epicsExportRegistrar(npcRegistrar);


