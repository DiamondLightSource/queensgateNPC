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
#include <TakeLock.h>
#include <FreeLock.h>

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
        , connected(false)
        , _pollCounter(SLOW_POLL_FREQ_CONST)
{
    printf("creating QgateAxis %d '%s'\n", axisNumber, axisName);
    
    //XXX: addAxis really needed?
    controller.addAxis(axisNumber);
    
    setIntegerParam(ctrler.motorStatus_, 0);
    setIntegerParam(ctrler.motorStatusCommsError_, 1);
    //TODO: move initAxis to poll for delayed init?
    // connected = initAxis();
    initAxis();
}

QgateAxis::~QgateAxis() {}

bool QgateAxis::initAxis() {
    std::string value;
    bool result = false;
    
    //EPICS is not in charge of the closed loop operation, it is the Queensgate controller,
    // but nevertheless it is a closed loop.
    setClosedLoop(true);
    setIntegerParam(ctrler.motorStatusFollowingError_, 0);
    setDoubleParam(ctrler.profileFollowingErrors_, 0.0);

    //XXX:
    printf("]]]]]]]]]]]]]]]initAxis %d\n", axisNum);

    result = getStatusConnected();
    if(result) {
        ctrler.getCmd("identity.stage.part.get", axisNum, value);
        setStringParam(ctrler.QG_AxisModel, value.c_str());
    }
    
    //XXX:
    // printf("]]]]]]]]]]]]]]]axis %d initially %sconnected\n", axisNum, (result)? "":"DIS");

    return result;
}

/** Poll the axis, start moves when possible, etc.  This function is
 * entered with the lock already on.
 * \param[out] moving Set to TRUE if the axis is moving
 */
asynStatus QgateAxis::poll(bool *moving) {
    bool result = false;
    bool wasconnected = connected;

    //XXX:printf
    printf("%s:\tAxis %d (%d) STATUS=%d polling: %sconnected\n", mytime(), 
            this->axisNum, this->axisNo_, status_.status, (connected)? "":"DIS" );
    
    if(ctrler.connected) {
        //fast poll
        result |= getStatusConnected();
        if(result) {
            result |= getPosition();    
            *moving = !getStatusMoving();
        }
        //slow poll
        if (_pollCounter++ % SLOW_POLL_FREQ_CONST == 0) {

            // XXX:
            // printf("%s:SLOWPOLL}}}}}}}}}}}}}}}}}\n", mytime());

            result |= getAxisMode();
        }
    } else {
        //TODO: Axis reconnection poll (slow)
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
            *moving = false;    //Just disconnected
        } else {
            initAxis();         //Been re-connected
        }
    }
    callParamCallbacks();
    return asynSuccess;   
}

bool QgateAxis::getStatusConnected() {
    bool result = false;    //Default for when controller not connected

    //XXX: 
    // printf(":::::::::Checking if axis %d connected (CTRL=%d)\n", axisNum, ctrler.connected);

    if(ctrler.connected) {
        std::string value;
        if(ctrler.getCmd("stage.status.stage-connected.get", axisNum, value) == DLL_ADAPTER_STATUS_SUCCESS) {
            result = atoi(value.c_str());
        }
    }
    if(result != connected) {
        //Update on change

        //XXX: 
        printf(":::::::::axis %d change on connected status (%d)\n", axisNum, result);
        int mystatus, myce;
        mystatus = ctrler.getIntegerParam(axisNo_, ctrler.motorStatus_, &myce);
        printf(":::::::::STATUS %d (%d) commsErr=%d\n", mystatus, status_.status, myce);

        connected = result;
        //Update status bits
        setIntegerParam(ctrler.motorStatusProblem_, !connected);
        setIntegerParam(ctrler.motorStatusCommsError_, !connected);
        //Update status of related PVs
        setIntegerParam(ctrler.QG_AxisConnected, connected);

        //XXX: 
        // printf(":::::::::axis %d is now %sconnected (%d)\n", axisNum, (result)?"":"DIS", result);
                
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

bool QgateAxis::getInLPFPosition() {
    std::string value;
    if(ctrler.getCmd("stage.status.in-position.lpf-confirmed.get", axisNum, value) == DLL_ADAPTER_STATUS_SUCCESS) {
        setIntegerParam(ctrler.QG_AxisInPosLPF, atoi(value.c_str()));
        // TODO: setPosition() call here?
        return true;
    }
    return false;
}
bool QgateAxis::getStatusMoving() {
    //TODO: use unconfirmed or LPF position according to motor record config?
    bool inPos = getInLPFPosition();
    setIntegerParam(ctrler.motorStatusDone_, inPos);
    setIntegerParam(ctrler.motorStatusMoving_, !inPos);
    return inPos;
}

bool QgateAxis::getPosition() {
    std::string value;
    if(ctrler.getCmd("stage.position.measured.get", axisNum, value) != DLL_ADAPTER_STATUS_SUCCESS) {
        return false;
    }
    double positionMicrons = PM_TO_MICRONS(atof(value.c_str()));
    double position = atof(value.c_str());
    asynPrint(pasynUser_, ASYN_TRACE_FLOW, "Axis %d measured pos=%lf microns (%lf pm)\n", 
                axisNum, positionMicrons, position);

    //XXX:printf
    printf("/-/-/-/-/-/Stage1 measured pos=%lf microns (%lf pm)\n", positionMicrons, position);
    
    // setDoubleParam(QG_AxisPos, position);
    setDoubleParam(ctrler.motorEncoderPosition_, position);
    setDoubleParam(ctrler.motorPosition_, position);
    return true;
}

#ifdef MOVEAXIS
/** Move the motor to an absolute location or by a relative amount.
  * \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move 
  * by (if relative=1). Units=steps.
  * \param[in] relative  Flag indicating relative move (1) or absolute move (0).
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus QgateAxis::move(double position, int relative,
            double minVelocity, double maxVelocity, double acceleration) {
    if(!connected) {
        // If axis not connected to a stage, ignore
        return asynSuccess;
    }

    //XXX:
    printf("%s:MOOOOOOVEEEEEEE!!!!! QgateNPC.move: pos=%lf, relative=%d, minV=%lf, maxV=%lf, acc=%lf\n",
            mytime(), position, relative,
            minVelocity, maxVelocity, acceleration);

    // Start the move
	TakeLock takeLock(&ctrler, /*alreadyTaken=*/true);
    FreeLock freeLock(takeLock);

    //Note: NPC controller have pre-configured movement parameters (e.g. velocity, accel)
    if(ctrler.moveCmd("stage.position.absolute-command.set", axisNum, position) != DLL_ADAPTER_STATUS_SUCCESS) {
        //XXX:
        printf("SE JODIO LA VAINA !!!!\n");
        return asynError;
    }       
    //XXX:
    else {
        printf("COMMANDED TO MOVE !!!!\n");
    }

    if(relative) {
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
#endif
    return asynSuccess;
}

// /** Jog axis command
//  * \param[in] minVelocity The minimum velocity during the move
//  * \param[in] maxVelocity The maximum velocity during the move
//  * \param[in] acceleration The acceleration to use
//  */
// asynStatus QgateAxis::moveVelocity(double minVelocity,
//         double maxVelocity, double acceleration) {
//     //TODO: what to do here for jogging? call move repeatedly relative from the current pos?
//     printf("COMMANDED TO JOG: v=[%lf, %lf], acc=%lf\n", minVelocity, maxVelocity, acceleration);
//     return asynSuccess;
// }

// /** Set the current position of the motor.
//   * \param[in] position The new absolute motor position that should be set in the hardware. Units=steps.*/
// asynStatus QgateAxis::setPosition(double position) {
//     if(!connected) {
//         // If axis not connected to a stage, ignore
//         return asynSuccess;
//     }
//     //TODO: what are we supposed to do here?
//     printf("++++++++++++++++++++++++++++++++++++set POSITION to %lf, yes\n", position);

//     return asynSuccess;
// }

asynStatus QgateAxis::stop(double acceleration) {
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

    double position;
    if(!getPosition()) {
        //XXX:
        printf("ignoring stop...\n");
        return asynSuccess;
    }
    ctrler.getDoubleParam(axisNo_, ctrler.motorPosition_, &position);
    //XXX:
    printf("%s:SSTTOOOPP!!!!! at %lf\n", mytime(), position);
    //Note: NPC controller have pre-configured movement parameters (e.g. velocity, accel)
    if(ctrler.moveCmd("stage.position.absolute-command.set", axisNum, position) != DLL_ADAPTER_STATUS_SUCCESS) {
        //XXX:
        printf("SE JODIO LA VAINA del STOP !!!!\n");
        return asynError;
    }       
    //XXX:
    else {
        printf("COMMANDED TO STOP !!!!\n");
    }
    return asynSuccess;
}

int QgateAxis::physicalAxis() {
	return axisNum;
}

bool QgateAxis::isConnected() {
    return connected;
}
