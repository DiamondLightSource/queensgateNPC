#include <sstream>
#include <stdlib.h>
#include <string.h>
/*
#include <stddef.h>
#include <stdio.h>
#include <paramLib.h>
*/

#include <epicsExport.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>

#include "queensgateNPCcontroller.hpp"

#define QGATE_NUM_PARAMS 100

//XXX: timestamp for debug
std::string findQGList(QGList &qglist, const int position) {
    int pos=0;
    for(QGList::iterator it=qglist.begin(); it!=qglist.end(); ++it, ++pos) {
        if(pos==position) {
            return *it;
        }
    }
    return "Not Found";
}
void printQGList(QGList &qglist) {
    for(QGList::iterator it=qglist.begin(); it!=qglist.end(); ++it) {
        printf("%s,", (*it).c_str());
    }
    printf("\n");
}

//XXX: timestamp for debug
#include <sys/time.h>
char* mytime() {
    static char timeStamp[100];
    struct timeval tv;
    struct tm *tm;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    tm=localtime(&tv.tv_sec);
    // uint64_t timest=tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
    snprintf(timeStamp, 100, "%d:%02d:%02d.%ld", tm->tm_hour, tm->tm_min,
            tm->tm_sec, tv.tv_usec);
    return timeStamp;
}

/*
 * Driver object for communication with the controller
 */
QgateController::QgateController(const char *portName, 
                                const char* serialPortName,
                                const int maxNumAxes, 
                                double movingPollPeriod, 
                                double idlePollPeriod,
                                const char* libraryPath)
    : asynMotorController(portName, 
            maxNumAxes,
            QGATE_NUM_PARAMS,
            asynFloat64Mask | asynInt32Mask | asynOctetMask | asynDrvUserMask, /* Interface mask */
            asynFloat64Mask | asynInt32Mask | asynOctetMask, /* Interrupt mask */
            ASYN_MULTIDEVICE | ASYN_CANBLOCK, /* asynFlags */
            1, /* Autoconnect */
            0, /* Default priority */
            0) /* Default stack size */
    , numAxes(maxNumAxes)
    , maxAxes(QgateController::NOAXIS)
    , nameCtrl(portName)
    , initialised(false)
    , connected(false)
    , deferringMode(false)
{
    // Uncomment these lines to enable asyn trace flow and error
    // pasynTrace->setTraceMask(pasynUserSelf, 0xFF);
    // pasynTrace->setTraceIOMask(pasynUserSelf, 0xFF);
    // pasynTrace->setTraceInfoMask(pasynUserSelf, 0xFF);
    // pasynTrace->setTraceFile(pasynUserSelf, stdout);
    
    //printf("AsynParams lists: %d\n", asynParams.numLists());

    createParam(QG_CtrlStatusCmd,       asynParamOctet,     &QG_CtrlStatus);
    createParam(QG_CtrlConnectedCmd,    asynParamInt32,     &QG_CtrlConnected);
    createParam(QG_CtrlModelCmd,        asynParamOctet,     &QG_CtrlModel);
    createParam(QG_CtrlFirmwareCmd,     asynParamOctet,     &QG_CtrlFirmware);
    createParam(QG_CtrlSerialNumCmd,    asynParamOctet,     &QG_CtrlSerialNum);
    createParam(QG_CtrlMaxStagesCmd,    asynParamInt32,     &QG_CtrlMaxStages);
    createParam(QG_CtrlMaxAxesCmd,      asynParamInt32,     &QG_CtrlMaxAxes);
    createParam(QG_CtrlDLLverCmd,       asynParamOctet,     &QG_CtrlDLLver);
    createParam(QG_CtrlSecurityCmd,     asynParamOctet,     &QG_CtrlSecurity);
    createParam(QG_AxisNameCmd,         asynParamOctet,     &QG_AxisName);
    createParam(QG_AxisModelCmd,        asynParamOctet,     &QG_AxisModel);
    createParam(QG_AxisConnectedCmd,    asynParamInt32,     &QG_AxisConnected);
    createParam(QG_AxisModeCmd,         asynParamInt32,     &QG_AxisMode);
    createParam(QG_AxisPosCmd,          asynParamFloat64,   &QG_AxisPos);
    createParam(QG_AxisInPosLPFCmd,     asynParamInt32,     &QG_AxisInPosLPF);

    // Uncomment this line to display list of params at startup
    // printf("AsynParams lists: %d\n", asynParams.numLists());

    //print params
    // this->reportParams(stdout,2); 

    /* Initialise Prior's Library controller */
    if(initController(serialPortName, libraryPath) != asynSuccess) {
        printf("Failed to initialise %s controller\n", portName);
    }
    
    //XXX: not needed to connect asyn
    // Connect asyn to the serial port interface
    // int retries=3;
    // do {
    //     // if(pasynManager->connectDevice(pasynUserSelf, serialPortName, 0)
    //     //         != asynSuccess) {
    //     if(pasynOctetSyncIO->connect(serialPortName, 0,
    //             &serialPortUser, NULL) != asynSuccess) {
    //         printf("NPCcontroller: Failed to connect to serial port %s\n",
    //                 serialPortName);
    //         epicsThreadSleep(1);
    //     }
    //  // pasynOctetSyncIO->setInputEos(serialPortUser, "\n", 1);
    //  // pasynOctetSyncIO->setOutputEos(serialPortUser, "\n", 1);
    // } while (--retries>0);
    // if(retries<1) return;

    setIntegerParam(QG_CtrlMaxAxes, numAxes);
    initialised = true;

    //XXX: ?? // Synchronization variables
    // for (int i=0; i < MAX_N_REPLIES; i++) {
    //     replyEvents[i] = epicsEventCreate(epicsEventEmpty);
    // }

    /** Starts the motor poller thread.
     * \param[in] movingPollPeriod The time in secs between polls when any axis is moving.
     * \param[in] idlePollPeriod The time in secs between polls when no axis is moving.
     * \param[in] forcedFastPolls The number of times to force the movingPollPeriod after waking up the poller.  
     * This can need to be non-zero for controllers that do not immediately
     * report that an axis is moving after it has been told to start. */
    startPoller(movingPollPeriod, idlePollPeriod, /*forcedFastPolls-*/3);
}

QgateController::~QgateController() {
    // for (int i=0; i < MAX_N_REPLIES; i++) {
    //     epicsEventDestroy(replyEvents[i]);
    //     // epicsMutexDestroy(replyLocks[i]);
    // }
    //TODO: close all channels/axes before closing controller??
    //TODO: set shuttingDown_ ??
    qg.CloseSession();
}

asynStatus QgateController::initController(const char* portDevice, const char* libPath) {
    int dllVersionMajor, dllVersionMinor, dllVersionBuild;
    DllAdapterStatus result = DLL_ADAPTER_STATUS_SUCCESS;
    QGList listresName, listresVal;
    
    //XXX:
    printf("Initialising Controller: %s\n", libPath);

    //Init DLL: requires the name and path to the controller_interface64.so file
    result = qg.Init(libPath);
    if ( result != DLL_ADAPTER_STATUS_SUCCESS ) {
        printf("queensgateNPC: DLL file not found!\n");
        return asynError;
    }

    //XXX:
    printf("Initialising Controller Session %s on %s\n", nameCtrl.c_str(), portDevice);

    //Open controller session
    // Note: Controller emulator uses "sim:/NPCxxxx" format, e.g. "sim:/NPC6330" for the NPC6330 controller
    result = qg.OpenSession(portDevice);
    if ( result != DLL_ADAPTER_STATUS_SUCCESS ) {
        printf("queensgateNPC: DLL session not created! Error %d\n", result);
        return asynError;
    }

    /* Get non-mutable information */
    qg.GetDllVersion(dllVersionMajor, dllVersionMinor, dllVersionBuild);
    {
        char dll[100];
        snprintf(dll, 100, "%d.%d.%d", dllVersionMajor, dllVersionMinor, dllVersionBuild);
        setStringParam(QG_CtrlDLLver, dll);
        printf("queensgateNPC: Initialising SDK %s\n", dll);
    }
    return asynSuccess;
}

asynStatus QgateController::initialChecks() {
    DllAdapterStatus result = DLL_ADAPTER_STATUS_SUCCESS;
    QGList listresName, listresVal;

    maxAxes = qg.GetChannels();
    //Initialise/reset deferred move storage
    deferredMove.clear();
    deferredMove.reserve(maxAxes);
    for(int i=0; i<maxAxes; i++) {
        deferredMove.push_back("");
    }
    getCmd("identity.hardware.part.get", 0, model);
    getCmd("identity.hardware.serial.get", 0, serialNum);
    getCmd("identity.software.version.get", 0, ctrl_firmware);
    result = getCmd("controller.security.user.get", 0, securityLevel);
    if(securityLevel.compare("Queensgate user")!=0) {
        //TODO: set security level back to user -- this goes here or somewhere else?
        result = qg.DoCommand("controller.security.user.set 0xDEC0DED", listresName, listresVal);
        //TODO: check outcome
        getCmd("controller.security.user.get", 0, securityLevel);
    }

    if(result != DLL_ADAPTER_STATUS_SUCCESS) {
        return asynError;   //comms failed (again)
    }
    setStringParam(QG_CtrlModel, model.c_str());
    setStringParam(QG_CtrlSerialNum, serialNum.c_str());
    setStringParam(QG_CtrlFirmware, ctrl_firmware.c_str());
    setIntegerParam(QG_CtrlMaxStages, maxAxes);
    setIntegerParam(QG_CtrlMaxAxes, numAxes);
    setStringParam(QG_CtrlSecurity, securityLevel.c_str());
    
    //List all detected/connected stages, from 1 to maximum detected
    std::ostringstream reportTxt;
    reportTxt << "queensgateNPC Controller " << model << " " << nameCtrl <<
                " - S/N:" << serialNum << 
                " - up to " << maxAxes << " channels." << std::endl;
    for(int i=1; i<=maxAxes; ++i) {
        std::ostringstream stageCmd;
        stageCmd << "identity.stage.part.get " << i;
        result = qg.DoCommand(stageCmd.str(), listresName, listresVal);
        reportTxt << "Stage[" << i << "]:";
        if(result== DLL_ADAPTER_STATUS_SUCCESS) {
            //Controller reports non-connected stage as FAILED, and it sounds too dramatic
            if(listresVal.front().compare("FAILED")) {
                reportTxt << listresVal.front();
            } else {
                reportTxt << "Not found";    
            }
        } else {
            reportTxt << "Error accessing stage";
        }
        reportTxt << std::endl;
    }
    printf("%s", reportTxt.str().c_str());
    return asynSuccess;
}

/** Polls the controller and updates values
 */
asynStatus QgateController::poll() {
    TakeLock takeLock(this, /*alreadyTaken=*/true);
	FreeLock freeLock(takeLock);

    //get controller status
    //TODO: check security level change
    std::string reply;
    if(getCmd("controller.status.get", 0, reply, 2) != DLL_ADAPTER_STATUS_SUCCESS) {
        setIntegerParam(QG_CtrlConnected, 0);
        if(connected) {
            connected = false;
            //TODO: tell all the axes that the controller is not connected
            printf("QueensgateNPC: controller %s %s disconnected\n", model.c_str(), nameCtrl.c_str());
        }
    } else {
        if(!connected) {
            //Just re-connected to the controller
            setStringParam(QG_CtrlStatus, reply.c_str());
            if(initialChecks() != asynSuccess) {
                //Connection failed again
                setIntegerParam(QG_CtrlConnected, 0);
                return asynSuccess; //Controller disconnected: re-check on next polling
            }
            //XXX:
            printf("QueensgateNPC: controller %s %s connected\n", model.c_str(), nameCtrl.c_str());
            
            setIntegerParam(QG_CtrlConnected, 1);
            connected = true;
            if(getCmd("controller.status.get", 0, reply, 0) == DLL_ADAPTER_STATUS_SUCCESS) {
                setStringParam(QG_CtrlSecurity, reply.c_str());
            }
        }
    }

        //  QgateAxis *axis = dynamic_cast<QgateAxis *>(this->getAxis(pollAxis));
 
    //Uncomment this line for printing a long list of params each poll interval
    // this->reportParams(stdout,2);

    callParamCallbacks();
    return asynSuccess;
}

bool QgateController::isConnected() {
    std::string partID;  //Store result here
    if(getCmd("identity.hardware.part.get", 0, partID) == DLL_ADAPTER_STATUS_SUCCESS) {

        //XXX:
        printf("Check controller: %s\n", partID.c_str());    

        return true;
    } else {
        printf("no controller answers :(\n");
    }
    return false;   //Not found
}

/** Processes deferred moves.
  * \param[in] deferMoves defer moves till later (true) or process moves now (false) */
asynStatus QgateController::setDeferredMoves(bool deferMoves) {
    DllAdapterStatus result = DLL_ADAPTER_STATUS_ERROR_UNKNOWN_COMMAND;
    //coming from motorDeferMoves_
    printf("..................Commanded to %sDEFER MOVES (%d)\n", (deferMoves)?"":"Flush ", deferMoves);
    if(!deferMoves && deferringMode) {
        //TODO: Flush moves
        QGList listresName, listresVal;
        std::string syncMove;
                
        //XXX
        printdefmoves();

        for(int i=0; i<maxAxes; i++) {
            syncMove.append(deferredMove[i]);
            syncMove.append("\n");  //Separator between commands
            deferredMove[i].clear();
        }
            
        //XXX:
        printf("Deferred moves requested to Flush: '%s'\n", syncMove.c_str());

        deferringMode = false;
        result = qg.DoCommand(syncMove.c_str(), listresName, listresVal);
        //XXX:
        printQGList(listresName);
        printQGList(listresVal);

        if(result != DLL_ADAPTER_STATUS_SUCCESS) {
            //TODO asynPrint
            printf("Failed to execute deferred command: %d\n", result);
            return asynError;
        }
    }
    if (deferMoves) {
        //Enable deferring moves. Moves on all axes are delayed for later
        deferringMode = true;
    }
    return asynSuccess;
}

//XXX
void QgateController::printdefmoves() {
    for (unsigned int i=0;i<deferredMove.size();i++) {
        printf("\t[%d]=%s\n", i, deferredMove[i].c_str());
    }
}

DllAdapterStatus QgateController::moveCmd(std::string cmd, int axisNum, double value) {
    DllAdapterStatus result = DLL_ADAPTER_STATUS_ERROR_UNKNOWN_COMMAND;
    QGList listresName, listresVal;
    // Convert pm to microns
    // double nativeValue = MICRONS_TO_PM(value);
    
    // const char *driverName = "QGATE";
    // const char *functionName = "moveCmd";
    // printf("Function %s\n", functionName);
    //asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "Function %s=%d\n", functionName, function);
   
    std::ostringstream stageCmd;
    stageCmd << cmd << " " << axisNum << " " << std::fixed << value;

    if (deferringMode) {
        deferredMove[axisNum-1] = stageCmd.str(); //Only the last request is stored
        
        //XXX
        printdefmoves();

        return DLL_ADAPTER_STATUS_SUCCESS;
    }

    //XXX: remove multiple tries code
    // unsigned int numtries = 2;
    // do {
        //XXX:
        printf("Stage %d moving CMD:'%s'\n", axisNum, stageCmd.str().c_str());
        
        result = qg.DoCommand(stageCmd.str().c_str(), listresName, listresVal);
        //XXX:
        printQGList(listresName);
        printQGList(listresVal);
        
        //XXX:
        double resultMicrons = PM_TO_MICRONS(atof(listresVal.begin()->c_str()));
        double newValue = atof(listresVal.begin()->c_str());
        // printf("Stage %s (%d) requested to move req=%lf pos=%lf microns\n", 
        //             (result==DLL_ADAPTER_STATUS_SUCCESS)?"":"NOT", result, nativeValue, resultMicrons);
        printf("Stage %s (%d) requested to move req=%lf, moved=%lf (%lf microns)\n", 
                    (result==DLL_ADAPTER_STATUS_SUCCESS)?"":"NOT", result, value, newValue, resultMicrons);
    //     if (result!=DLL_ADAPTER_STATUS_SUCCESS) {
    //         epicsThreadSleep(0.5);
    //     }
    // } while( (result!=DLL_ADAPTER_STATUS_SUCCESS) && (numtries-- > 0) );
    return result;
}

DllAdapterStatus QgateController::getCmd(std::string cmd, int axisNum, std::string &value, int valueID) {
    DllAdapterStatus result = DLL_ADAPTER_STATUS_ERROR_UNKNOWN_COMMAND;
    QGList listresName, listresVal;
    // const char *driverName = "QGATE";
    // const char *functionName = "getCmd";
    //TODO: get pasynUser ?
    // asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "Function %s=%d\n", functionName, function);
    //XXX:
    // printf("Function %s\n", functionName);
   
    std::ostringstream stageCmd;
    stageCmd << cmd;
    if(axisNum > 0) {
        //Axis 0 is the controller itself and does not need this parameter
        stageCmd << " " << axisNum;
    }
    //XXX:
    // printf("%s:Stage %d requesting CMD:'%s'\n", 
    //         mytime(), axisNum, stageCmd.str().c_str());

    result = qg.DoCommand(stageCmd.str().c_str(), listresName, listresVal);
    //TODO: if valueID empty, get the 1st element on listResVal
    if(result==DLL_ADAPTER_STATUS_SUCCESS) {
        value = findQGList(listresVal, valueID);
        //XXX: printf("Stage %d request reply:'%s'\n", axisNum, value.c_str());
    } else {
        printf("Failed request: %d\n", result);
        value.clear();  //empty string
    }
    return result;
}

DllAdapter& QgateController::getAdapter() {
    return qg;
}

bool QgateController::isAxisPresent(int axisNum) {
    std::string value;  //Store result here
    if(getCmd("identity.stage.part.get", axisNum, value) == DLL_ADAPTER_STATUS_SUCCESS) {
        //Controller reports non-connected stage as "FAILED"
        if(value.compare("FAILED")) {
                //Stage connected
                return true;
            } 
    }
    return false;   //Not found
}

