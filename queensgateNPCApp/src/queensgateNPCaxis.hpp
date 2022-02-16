#ifndef QGATENPCaxis_H_
#define QGATENPCaxis_H_

#include <asynMotorController.h>
#include <asynMotorAxis.h>

// #include "include/controller_interface.h"
// #include "include/dll_adapter.hpp"
#include "queensgateNPCcontroller.hpp"

class QgateAxis : public asynMotorAxis 
{
public:
    QgateAxis(QgateController &controller,
                int axisNumber,
                const char *axisName
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
    int axisNum;    //Axis number for DLL [1..n]
                    //Note that it differs from asynMotorAxis::axisNo_ being the axis index [0..n]
    std::string axis_name;  //name of the stage
    std::string axis_model; //(Reported) model of the stage
    bool connected;
    unsigned int _pollCounter;  //Iteration counter for slow polling
    // bool axis_inPos;        //in position (LPF-confirmed)
    //TODO: configure In-position for unconfirmed/LPF/Window
    //TODO: separate PVs for unconfirmed/LPF/Window In-position 
private:
    bool initAxis();
    bool getStatusConnected();
    bool getStatusMoving(bool &moving);
    bool getAxisMode();
    bool getPosition();
    bool getInLPFPosition(bool &moving);
};

#endif //ONCE
