#ifndef QGATENPCaxis_H_
#define QGATENPCaxis_H_

#include <stdio.h>

#include <asynMotorController.h>
#include <asynMotorAxis.h>

// #include "include/controller_interface.h"
// #include "include/dll_adapter.hpp"
#include "queensgateNPCcontroller.hpp"

class QgateAxis : public asynMotorAxis 
{
public:
    enum AXISMODE {
        AXISMODE_NATIVE = 0,    //moving when not in position and not HV saturated
        AXISMODE_UNCONFIRMED = 1,
        AXISMODE_WINDOW = 2,    //In position by margin
        AXISMODE_LPF = 3,       //Low-pass filter (not oscillating)
        AXISMODE_BOTH = 4       //Confirmed by both Window and Low-pass filter
    };
    enum AXISTYPE {
        AXISTYPE_STAGE = 0,
        AXISTYPE_SENSOR = 1
    };
public:
    QgateAxis(QgateController &controller,
                unsigned int axisNumber,
                const char *axisName,
                unsigned int axisMode=AXISMODE_NATIVE,
                unsigned int axisType=AXISTYPE_STAGE
                );
    virtual ~QgateAxis();
public:
    // Overridden from asynMotorAxis
    virtual asynStatus poll(bool *moving);
    virtual asynStatus move(double position, int relative,
            double minVelocity, double maxVelocity, double acceleration);
    virtual asynStatus stop(double acceleration);
private:
    static const int SLOW_POLL_FREQ_CONST=8;
    QgateController& ctrler;
    DllAdapter& qg;  //Queensgate adapter
    unsigned int axisNum;    //Axis number for DLL [1..n]
                        //Note that it differs from asynMotorAxis::axisNo_ being the axis index [0..n]
    std::string axis_name;  //name of the stage
    std::string axis_model; //(Reported) model of the stage
    unsigned int axis_mode; //In-position mode to be used
    bool isSensor;          //Configured as sensor (no motion) or active stage
    //Status attributes
    bool initialStatus;     //Initial status, before first polling
    bool connected;         //Axis connected status
    bool forceStop;         //Stop status was forced
    unsigned int _pollCounter;  //Iteration counter for slow polling
    
private:
    bool initAxis();
    bool getStatusConnected();
    bool getStatusMoving(bool &moving);
    bool getAxisMode();
    bool getPosition();
    bool updateAxisPV(std::string sCmd, int indexPV);
};

#endif //ONCE
