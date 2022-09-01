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
    QgateAxis(QgateController &controller,
                unsigned int axisNumber,
                const char *axisName,
                unsigned char axisType=AXISTYPE_STAGE
                );
    virtual ~QgateAxis();
public:
    enum AXISTYPE {
        AXISTYPE_STAGE = 0,
        AXISTYPE_SENSOR = 1
    };
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
    bool getInPositionLPF();
    bool getInPositionUnconfirmed();
    bool getInPositionWindow();
};

#endif //ONCE
