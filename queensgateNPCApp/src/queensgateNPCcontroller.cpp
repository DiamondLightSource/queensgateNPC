#include <stdlib.h>
#include <string>
#include <sstream>
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

const char *driverName = "queensgateNPC";

/** Gets a DoCommand list's content from its position.
 * \param[in] position position in the list. */
std::string QGList::find(const int position) {
    int pos=0;
    for(QGList::iterator it=this->begin(); it!=this->end(); ++it, ++pos) {
        if(pos==position) {
            return *it;
        }
    }
    return "Not Found";
} 

/** Gets a DoCommand's full list of contents.
 */
std::string QGList::print() {
    std::ostringstream qgList;
    for(QGList::iterator it=this->begin(); it!=this->end(); ++it) {
        qgList << *it << ",";
    }
    return qgList.str();
}

/** Driver object for communication with the controller
 * \param[in] portName The asyn name.
 * TODO: finish this commen
 * \param[in] movingPollPeriod The time in secs between polls when any axis is moving.
 * \param[in] idlePollPeriod The time in secs between polls when no axis is moving.
 * \param[in] libraryPath The number of times to force the movingPollPeriod after waking up the poller.  
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
    , portDevice(serialPortName)
    , nameCtrl(portName)
    , initialised(false)
    , connected(false)
    , deferringMode(false)
{
    // Uncomment these lines to enable full asyn trace flow and error
    //pasynTrace->setTraceMask(pasynUserSelf, 0xFF);
    //pasynTrace->setTraceIOMask(pasynUserSelf, 0xFF);
    //pasynTrace->setTraceInfoMask(pasynUserSelf, 0xFF);
    //pasynTrace->setTraceFile(pasynUserSelf, stdout);
    //pasynTrace->setTraceInfoMask(pasynUserSelf, ASYN_TRACEINFO_TIME & ASYN_TRACEINFO_PORT);
    
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
    createParam(QG_AxisMovingCmd,       asynParamInt32,     &QG_AxisMoving);
    createParam(QG_AxisInPosUnconfirmedCmd, asynParamInt32,    &QG_AxisInPosUnconfirmed);
    createParam(QG_AxisInPosLPFCmd,     asynParamInt32,     &QG_AxisInPosLPF);
    createParam(QG_AxisInPosWindowCmd,  asynParamInt32,     &QG_AxisInPosWindow);

    bool initialStatus = true;  //Assume controller would be initialised
    bool failedDLL = false;     //DLL initialisation (severe error)

    // Uncomment this line to display list of params at startup
    // this->reportParams(stdout,2); 

    /* Initialise Prior's Library controller */
    if(initController(libraryPath) != asynSuccess) {
        printf("Failed to initialise %s controller\n", portName);
        //TODO: set all offline
        setIntegerParam(QG_CtrlConnected, 0);
        initialStatus = false;
        failedDLL = true;   //DLL failed initialisation: can't recover from this!
    }
    
    /* Initialise Prior's controller session */
    if(initSession() != asynSuccess) {
        printf("Failed to initialise %s controller session\n", portName);
        //TODO: set all offline
        setIntegerParam(QG_CtrlConnected, 0);
        initialStatus = false;
    }

    setIntegerParam(QG_CtrlMaxAxes, numAxes);
    initialised = initialStatus;

    if(!failedDLL) {
        /** Starts the motor poller thread.
         * \param[in] movingPollPeriod The time in secs between polls when any axis is moving.
         * \param[in] idlePollPeriod The time in secs between polls when no axis is moving.
         * \param[in] forcedFastPolls The number of times to force the movingPollPeriod after waking up the poller.  
         * This can need to be non-zero for controllers that do not immediately
         * report that an axis is moving after it has been told to start. */
        startPoller(movingPollPeriod, idlePollPeriod, /*forcedFastPolls-*/3);
    }
}

QgateController::~QgateController() {
    qg.CloseSession();
}

asynStatus QgateController::initController(const char* libPath) {
    DllAdapterStatus result = DLL_ADAPTER_STATUS_SUCCESS;
    
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "Initialising Controller Library: %s\n", libPath);

    //Init DLL: requires the name and path to the controller_interfaceXX.so file
    result = qg.Init(libPath);
    if ( result != DLL_ADAPTER_STATUS_SUCCESS ) {
        printf("queensgateNPC: FATAL ERROR! not found or incorrect DLL file: %s\n", libPath);
        return asynError;
    }
    return asynSuccess;
}

asynStatus QgateController::initSession() {
    int dllVersionMajor, dllVersionMinor, dllVersionBuild;
    DllAdapterStatus result = DLL_ADAPTER_STATUS_SUCCESS;
    QGList listresName, listresVal;

    printf("Initialising Controller Session %s on %s\n", nameCtrl.c_str(), portDevice.c_str());

    //Open controller session
    // Note: Controller emulator uses "sim:/NPCxxxx" format, e.g. "sim:/NPC6330" for the NPC6330 controller
    result = qg.OpenSession(portDevice);
    if ( result != DLL_ADAPTER_STATUS_SUCCESS ) {
        printf("queensgateNPC: DLL session not created! Error %d\n", result);
        return asynError;
    }

    /* Get non-mutable information */
    qg.GetDllVersion(dllVersionMajor, dllVersionMinor, dllVersionBuild);
    if(dllVersionMajor<0) { 
        printf("queensgateNPC: No DLL version function available!\n"); 
        }
    else {
        char dll[100];
        snprintf(dll, 100, "%d.%d.%d", dllVersionMajor, dllVersionMinor, dllVersionBuild);
        setStringParam(QG_CtrlDLLver, dll);
        asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "queensgateNPC: Initialising SDK %s\n", dll);
        printf("%s: Initialising SDK %s\n",driverName, dll);
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
        //TODO: check outcome changed
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
    std::string reply;
    if(getCmd("controller.status.get", 0, reply, 2) != DLL_ADAPTER_STATUS_SUCCESS) {
        setIntegerParam(QG_CtrlConnected, 0);
        if(connected) {
            connected = false;
            asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "QueensgateNPC: controller %s %s disconnected\n", model.c_str(), nameCtrl.c_str());
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
            asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "QueensgateNPC: controller %s %s connected\n", model.c_str(), nameCtrl.c_str());
            setIntegerParam(QG_CtrlConnected, 1);
            connected = true;
            if(getCmd("controller.status.get", 0, reply, 0) == DLL_ADAPTER_STATUS_SUCCESS) {
                setStringParam(QG_CtrlSecurity, reply.c_str());
            }
        } else {
            //TODO: check security level change
            getCmd("controller.security.user.get", 0, securityLevel);
            if(securityLevel.compare("Queensgate user")!=0) {
                //TODO: set security level back to user -- this goes here or somewhere else?
                QGList listresName, listresVal;
                qg.DoCommand("controller.security.user.set 0xDEC0DED", listresName, listresVal);
                //TODO: check outcome
                getCmd("controller.security.user.get", 0, securityLevel);
            }
        }
    }

    //Uncomment this line for printing a long list of params each poll interval
    // this->reportParams(stdout,2);

    callParamCallbacks();
    return asynSuccess;
}

/** Processes deferred moves.
  * \param[in] deferMoves defer moves till later (true) or process moves now (false) */
asynStatus QgateController::setDeferredMoves(bool deferMoves) {
    DllAdapterStatus result = DLL_ADAPTER_STATUS_ERROR_UNKNOWN_COMMAND;
    //coming from motorDeferMoves_
    
    asynPrint(pasynUserSelf, ASYN_TRACEIO_FILTER, ".....Commanded to %sDEFER MOVES (%d)\n", (deferMoves)?"":"Flush ", deferMoves);
    if(!deferMoves && deferringMode) {
        //Requesting to Flush moves
        QGList listresName, listresVal;
        std::string syncMove;
                
        printdefmoves();

        //Compose deferred move message for the controller
        for(int i=0; i<maxAxes; i++) {
            syncMove.append(deferredMove[i]);
            syncMove.append("\n");  //Separator between commands
            deferredMove[i].clear();
        }
            
        asynPrint(pasynUserSelf, ASYN_TRACEIO_FILTER, "Deferred moves requested: '\n%s'\n", syncMove.c_str());

        deferringMode = false;
        result = qg.DoCommand(syncMove.c_str(), listresName, listresVal);
        if(result != DLL_ADAPTER_STATUS_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Failed to execute deferred command: %d\n", result);
            return asynError;
        }
    }
    if (deferMoves) {
        //Enable deferring moves mode. Move requests on all axes are delayed until commanded
        deferringMode = true;
    }
    return asynSuccess;
}

void QgateController::printdefmoves() {
    for (unsigned int i=0;i<deferredMove.size();i++) {
        asynPrint(pasynUserSelf, ASYN_TRACEIO_FILTER, "\t[%d]=%s\n", i, deferredMove[i].c_str());
    }
}

DllAdapterStatus QgateController::moveCmd(std::string cmd, int axisNum, double value) {
    DllAdapterStatus result = DLL_ADAPTER_STATUS_ERROR_UNKNOWN_COMMAND;
    QGList listresName, listresVal;
    std::ostringstream stageCmd;    //Used for composing full command
   
    //Compose move command for controller
    stageCmd << cmd << " " << axisNum << " " << std::fixed << value;

    if (deferringMode) {
        //Store the request
        deferredMove[axisNum-1] = stageCmd.str(); //Only the last request is stored
        printdefmoves();        //Print current list of moves
        return DLL_ADAPTER_STATUS_SUCCESS;
    }

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Stage %s-%d moving CMD:'%s'\n", nameCtrl.c_str(), axisNum, stageCmd.str().c_str());
    result = qg.DoCommand(stageCmd.str().c_str(), listresName, listresVal);

    if(result==DLL_ADAPTER_STATUS_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Stage %s-%d request's reply:'%s'\n", nameCtrl.c_str(), axisNum, listresVal.print().c_str());
        //XXX:
        double resultMicrons = PM_TO_MICRONS(atof(listresVal.begin()->c_str()));
        double newValue = atof(listresVal.begin()->c_str());
        asynPrint(pasynUserSelf, ASYN_TRACEIO_FILTER, "Stage %s-%d %s requested to move req=%lf, moved=%lf (%lf microns)\n", 
                    nameCtrl.c_str(), axisNum, (result==DLL_ADAPTER_STATUS_SUCCESS)?"":"NOT", value, newValue, resultMicrons);
    } else {
        std::ostringstream errorStr;
        qg.GetErrorText(errorStr, result);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Stage %s-%d Failed request %d: %s --> %s\n", nameCtrl.c_str(), axisNum, result, errorStr.str().c_str(), stageCmd.str().c_str());
    }
    return result;
}

DllAdapterStatus QgateController::getCmd(std::string cmd, int axisNum, std::string &value, int valueID) {
    DllAdapterStatus result = DLL_ADAPTER_STATUS_ERROR_UNKNOWN_COMMAND;
    QGList listresName, listresVal;
    std::ostringstream stageCmd;    //Used for composing full command

    //Compose command for controller
    stageCmd << cmd;
    if(axisNum > 0) {
        //Axis 0 is the controller itself and does not need this parameter
        stageCmd << " " << axisNum;
    }
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Stage %s-%d requesting CMD:'%s'\n", nameCtrl.c_str(), axisNum, stageCmd.str().c_str());
    result = qg.DoCommand(stageCmd.str().c_str(), listresName, listresVal);
    //TODO: if valueID empty, get the 1st element on listResVal
    if(result==DLL_ADAPTER_STATUS_SUCCESS) {
        value = listresVal.find(valueID);
        asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Stage %s-%d request's reply:'%s'\n", nameCtrl.c_str(), axisNum, value.c_str());
    } else {
        std::ostringstream errorStr;
        qg.GetErrorText(errorStr, result);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Stage %s-%d Failed request %d: %s --> %s\n", nameCtrl.c_str(), axisNum, result, errorStr.str().c_str(), stageCmd.str().c_str());
        value.clear();  //return empty string
    }
    return result;
}

//TODO: DllAdapterStatus QgateController::sendCmd(std::string cmd, int axisNum, std::string &value, int valueID) {



DllAdapter& QgateController::getAdapter() {
    return qg;
}

bool QgateController::isAxisPresent(int axisNum) {
    std::string value;  //Store result here
    if(getCmd("identity.stage.part.get", axisNum, value) == DLL_ADAPTER_STATUS_SUCCESS) {
        //THe Queensgate Controller reports non-connected stage as "FAILED"
        if(value.compare("FAILED")) {
                //Stage connected
                return true;
            } 
    }
    return false;   //Not found
}

