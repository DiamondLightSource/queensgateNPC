#include <sstream>
#include <stdlib.h>
#include <string.h>

#include <epicsExport.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>

#include "queensgateNPCaxis.hpp"

#define QGATE_NUM_PARAMS 100

/*
 * Driver object for stage (axis) control
 */
QgateAxis::QgateAxis(QgateController &controller,
                    int axisNumber,
                    const char *axisName) :
        asynMotorAxis(&controller, axisNumber-1) //axis Num [1..n] converted to Index [0..n]
        , ctrler(controller)
        , qg(controller.getAdapter())
        , axisNum(axisNumber)
        , axis_name(axisName)
        , moveTimeout(10)
        , initialMoving(false)
        , connected(false)
        , _pollCounter(SLOW_POLL_FREQ_CONST)
{
    // printf("creating QgateAxis %d '%s'\n", axisNumber, axisName);
    
    //setIntegerParam(ctrler.motorStatus_, 0);
    //setIntegerParam(ctrler.motorStatusCommsError_, 1);
}

QgateAxis::~QgateAxis() {}

bool QgateAxis::initAxis() {
    std::string value;
    bool result = false;
    
    //EPICS is not in charge of the closed loop operation, it is the Queensgate controller,
    // but nevertheless it is a closed loop.
    setClosedLoop(true);
    setIntegerParam(ctrler.motorStatusFollowingError_, 0);

    result = getStatusConnected();
    if(result) {
        ctrler.getCmd("identity.stage.part.get", axisNum, value);
        setStringParam(ctrler.QG_AxisModel, value.c_str());
        initialMoving = true;
    }
    return result;
}

/** Poll the axis, start moves when possible, etc.  This function is
 * entered with the lock already on.
 * \param[out] moving Set to TRUE if the axis is moving
 */
asynStatus QgateAxis::poll(bool *moving) {
    bool result = false;
    bool wasconnected = connected;

    //Update initial staus of moving
    if(initialMoving) {
        initialMoving = false;
        *moving = false;
        //XXX
        aprintf("%s %s START POLL\n", mytime(), axis_name.c_str());
        return asynSuccess;   //First run of the poll
    }

    //XXX:printf
    aprintf("%s:\tAxis %d (%d) %s polling: %sconnected \tSTATUS=%d\n", mytime(), this->axisNum, this->axisNo_, axis_name.c_str(), (connected)? "":"DIS", status_.status);

    if(ctrler.connected) {
        //fast poll
        result |= getStatusConnected();
        if(result) {
            result |= getPosition();    
            result |= getStatusMoving(*moving);
        }
        //slow poll
        if (_pollCounter++ % SLOW_POLL_FREQ_CONST == 0) {
            result |= getAxisMode();
        }
    } else {
        //Axis reconnection poll (slow)
        *moving = false;
        //Slow poll
        if (_pollCounter++ % SLOW_POLL_FREQ_CONST == 0) {
            result |= getStatusConnected();     //Update Axis status
        }
    }
    //TODO: check !result and if != connected then failed after getStatusConnected()call
    //TODO: check !result and log it
    //Detect connection state change again for additional actions
    if(wasconnected != connected) {
        if(wasconnected) {
            //Just lost connection
            *moving = false;    
            forceStop = false;
        } else {
            initAxis();         //Been re-connected
        }
    }
    callParamCallbacks();
    return asynSuccess;   
}

bool QgateAxis::getStatusConnected() {
    bool result = false;    //Default for when controller not connected

    if(ctrler.connected) {
        std::string value;
        if(ctrler.getCmd("stage.status.stage-connected.get", axisNum, value) == DLL_ADAPTER_STATUS_SUCCESS) {
            result = atoi(value.c_str());
        }
    }
    if(result != connected) {
        //Update on change

        //XXX: 
        printf(":::::::::%s %s-%d change on connected status %sconnected\n", mytime(), ctrler.nameCtrl.c_str(), axisNum, (result)?"":"DIS");
        int mystatus, myce;
        mystatus = ctrler.getIntegerParam(axisNo_, ctrler.motorStatus_, &myce);
        printf(":::::::::%s %s-%d STATUS %d (%d) commsErr=%d\n", mytime(), ctrler.nameCtrl.c_str(), axisNum, mystatus, status_.status, myce);

        connected = result;
        //Update status bits
        ctrler.setIntegerParam(axisNo_, ctrler.motorStatusProblem_, !connected);
        ctrler.setIntegerParam(axisNo_, ctrler.motorStatusCommsError_, !connected);
        setIntegerParam(ctrler.motorStatus_, !connected);
        setIntegerParam(ctrler.motorStatusProblem_, !connected);
        setIntegerParam(ctrler.motorStatusCommsError_, !connected);
        
        //Update status of related PVs
        setIntegerParam(ctrler.QG_AxisConnected, connected);
    }
    return connected;
}

bool QgateAxis::getAxisMode() {
    std::string value;
    if(ctrler.getCmd("stage.mode.digital-command.get", axisNum, value) == DLL_ADAPTER_STATUS_SUCCESS) {
        setIntegerParam(ctrler.QG_AxisMode, atoi(value.c_str()));
        return true;
    }
    return false;
}

bool QgateAxis::getInLPFPosition(int &inPos) {
    bool result = false;   //feedback from controller failed
    std::string value;
    if(ctrler.getCmd("stage.status.in-position.lpf-confirmed.get", axisNum, value) == DLL_ADAPTER_STATUS_SUCCESS) {
        inPos = atoi(value.c_str());
        result = true;      //success getting position status
        forceStop = false;  //Already in position: no need to force stop
    }
    return result;   //failed
}

bool QgateAxis::getStatusMoving(bool &moving) {
    bool result = false;    //Assume feedback from controller failed
    int inPos = 0;          //Assume not in position

    //TODO: use unconfirmed or LPF position according to motor record config?
    result = getInLPFPosition(inPos);

    if(forceStop) {
            //XXX:
            printf("force stop at getStatusMoving\n");
        forceStop = false;
        inPos = 1;  //Stop forced
        result = true; //Confirmed
    }
    //Time out check
    if(!inPos) {
        if(chronox.stop() < moveTimeout) {

            //XXX:
            if(moving) aprintf("%s awaiting end of move\n", axis_name.c_str());

        }
        else {

            //XXX:
            aprintf("move timed out: force stop\n");

            inPos = 1;  //Stop forced
            forceStop = false;
            result = true; //Confirmed
        }
    }
    //Update Motor Record status
    setIntegerParam(ctrler.QG_AxisInPosLPF, inPos);
    setIntegerParam(ctrler.motorStatusDone_, inPos);
    setIntegerParam(ctrler.motorStatusMoving_, !inPos);
    moving = !inPos;

    return result;
}

bool QgateAxis::getPosition() {
    std::string value;
    if(ctrler.getCmd("stage.position.measured.get", axisNum, value) != DLL_ADAPTER_STATUS_SUCCESS) {
        return false;
    }
    double positionMicrons = PM_TO_MICRONS(atof(value.c_str()));
    double position = atof(value.c_str());
    asynPrint(pasynUser_, ASYN_TRACE_FLOW, "Queensgate %s Axis %d measured pos=%lf microns (%lf pm)\n", 
                ctrler.nameCtrl.c_str(), axisNum, positionMicrons, position);

    //XXX:printf values
    // printf("/-/-/-/-/-/Stage %s %d measured pos=%lf microns (%lf pm)\n", 
    //         ctrler.nameCtrl.c_str(), axisNum, positionMicrons, position);
    
    // setDoubleParam(QG_AxisPos, position);
    setDoubleParam(ctrler.motorEncoderPosition_, position);
    setDoubleParam(ctrler.motorPosition_, position);
    return true;
}

/** Move the motor to an absolute location or by a relative amount.
  * \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move 
  * by (if relative=1). Units=microns.
  * \param[in] relative  Flag indicating relative move (1) or absolute move (0). [NOT YET IMPLEMENTED in this method]
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec. [IGNORED in this method]
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec. [IGNORED in this method]
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. [IGNORED in this method]  */
asynStatus QgateAxis::move(double position, int relative,
            double minVelocity, double maxVelocity, double acceleration) {
    if(!connected) {
        // If axis not connected to a stage, ignore
        return asynSuccess;
    }

    //XXX:
    // printf("%s:MOOOOOOVEEEEEEE!!!!! QgateNPC.move: pos=%lf, relative=%d, minV=%lf, maxV=%lf, acc=%lf\n",
    //         mytime(), position, relative,
    //         minVelocity, maxVelocity, acceleration);

    // Start the move
	TakeLock takeLock(&ctrler, /*alreadyTaken=*/true);
    FreeLock freeLock(takeLock);

    //Note: NPC controller have pre-configured movement parameters (e.g. velocity, accel)
    if(ctrler.moveCmd("stage.position.absolute-command.set", axisNum, position) != DLL_ADAPTER_STATUS_SUCCESS) {
        //TODO: log error
        //XXX:
        printf("COMMAND FAILED ON THE CONTROLLER !!!!\n");
        
        //TODO: move this to centralized status
        connected = false;
        ctrler.setIntegerParam(axisNo_, ctrler.motorStatusCommsError_, !connected);
        
        return asynError;
    }       
    //XXX:
    else {
        printf("COMMANDED TO MOVE !!!!\n");
        forceStop = false;  //Cancel any previous stop request
        chronox.start();    //Start counting for timeout
    }

    if(relative) {
        //TODO: relative move
    //     ctrler.moveCmd("stage.position.command.set", physicalAxis(), position);
    //     //ctrler.doMove(physicalAxis(), position);
    // } else {
    //     ctrler.moveCmd("stage.position.absolute-command.set", physicalAxis(), position);
    // }
    // setIntegerParam(ctrler.motorStatusDone_, 0);
    // setIntegerParam(ctrler.motorStatusDirection_, (int)position > curPosition);

    // Wait for move to complete. 
    // std::string sstat; 
    // asynStatus status = asynError;
    // int curStatus;
    // bool moving = true;
    // bool inPos;
    // do {
    //     TakeLock again(freeLock);
    //     inPos = getStatusMoving();
    // //     result = ctrler.getCmd("stage.status.in-position.lpf-confirmed.get", 
    // //                             physicalAxis(), sstat);
    // //     if(result != DLL_ADAPTER_STATUS_SUCCESS)
    // //     {
    // //         printf("QgateNPC: Error checking moving status\n");
    // //         moving = false;
    // //     } else {
    // //         curStatus = atoi(sstat.c_str());
    // //         moving = !(curStatus == 3 || curStatus == 0);
    // //         if (!moving) {
    // //             status = asynSuccess;

    // //         }
    // //         //TODO: delay here to avoid hammering
    // //         //TODO: timeout of full move
    // //     }
    // } while(!inPos);

    // // Do a poll now
    // // pollStatus(freeLock);
    // return status;
    }
    return asynSuccess;
}

asynStatus QgateAxis::stop(double acceleration) {
    asynStatus status = asynError;
    if(!connected) {
        // If axis not connected to a stage, ignore
        return asynSuccess;
    }

    //TODO: clear axis' pending deferred command
    if(ctrler.deferringMode) {
        ctrler.deferredMove[axisNo_].clear();
        return asynSuccess;
    }

    //TODO: re-command the axis to be in the current position
    // Start the stop procedure
	TakeLock takeLock(&ctrler, /*alreadyTaken=*/true);
    FreeLock freeLock(takeLock);
    forceStop = true;
    double newPosition;
    if(!getPosition()) {
        
        //XXX:
        printf("ignoring stop...\n");

        return asynSuccess;
    }
    ctrler.getDoubleParam(axisNo_, ctrler.motorPosition_, &newPosition);
    //XXX:
    printf("%s:SSTTOOOPP!!!!! at raw pos %lf\n", mytime(), newPosition);
    //Note: NPC controller have pre-configured movement parameters (e.g. velocity, accel)
    if(ctrler.moveCmd("stage.position.absolute-command.set", axisNum, newPosition) != DLL_ADAPTER_STATUS_SUCCESS) {
        //XXX:
        printf("FAILED STOP COMMAND !!!!\n");
        status = asynError;
    }       
    //XXX:
    else {
        printf("COMMANDED TO STOP !!!!\n");

        status = asynSuccess;
    }

    return status;
}
