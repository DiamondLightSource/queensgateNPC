#ifndef QGATENPCcontroller_H_
#define QGATENPCcontroller_H_

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <TakeLock.h>
#include <FreeLock.h>

#include "controller_interface.h"
#include "dll_adapter.hpp"

//Convert native picometres to micrometres
#define PM_TO_MICRONS(value)    ((value) * 1.0e-6 )
#define MICRONS_TO_PM(value)    ((value) * 1.0e6 )

/*
 * command strings (basic for operation; not intended for advanced configuration nor calibration):
 *  controller.status.get                   Controller status
                    reply: (security,channels,status)=(Queensgate production,3,0x0000)
 *  identity.hardware.part.get              Controller part number
                    reply: (part)=(NPC6330)
 *  identity.hardware.serial.get            Controller serial number
                    reply: (serial)=(102505)
 *  identity.software.version.get           Controller FW version number
                    reply: (version) = (100990979)
 *  controller.security.user.get <code>     User level code=0xDEC0DED
                    reply: (security)=(Queensgate production)
 *  controller.security.user.set <code>     User level code=0xDEC0DED, needed for commanding positions
 *  stage.status.stage-connected.get <channel>      Stage connected and ready to use
                    reply: (value)=(1)
 *  identity.stage.part.get <channel>       Stage part number
                    reply: (part)=(NPS-X-15A)
 *  stage.position.absolute-command.get <channel:INT>                   Commanded absolute position
                    reply: (value)=(0.000000000e+00)
 *  stage.position.absolute-command.set <channel:INT> <value:FLOAT>     Set position desired value. Values in picometres
 *  stage.position.command.get <channel>            Commanded Relative position
                     reply: (value)=(0.000000000e+00)
 *  stage.position.command.set <channel> <value>    Relative position
 *  stage.position.measured.get <channel>           Stage measured position
                    reply: (value)=(3.900000000e+01)
 *  stage.status.in-position.unconfirmed.get <channel>              Stage unconfirmed in position
                    reply: (value)=(1) 
 *  stage.status.in-position.lpf-confirmed.get <channel>            Stage “in position” state confirmed by LPF algorithm
                    reply: (value)=(1)
 *  stage.status.in-position.window-filter-confirmed.get <channel>  Stage “in position” state confirmed by window filter algorithm
                    reply: (value)=(1)
    stage.status.stage-moving.get <channel>                         Stage moving status
                    reply: (value)=(1)      [0 or 1]
    stage.mode.digital-command.get <channel>      Stage control mode (digital means from PC/serial/USB)
                    reply: (value)=(1)      [0 or 1]
 */

/* EPICS asyn Commands */
#define QG_CtrlConnectedCmd         "QGATE_CONNECTED"
#define QG_CtrlStatusCmd            "QGATE_STATUS"
#define QG_CtrlModelCmd             "QGATE_MODEL"
#define QG_CtrlFirmwareCmd          "QGATE_FIRMWARE"
#define QG_CtrlSerialNumCmd         "QGATE_SERIALNO"
#define QG_CtrlMaxStagesCmd         "QGATE_MAXSTAGES"
#define QG_CtrlMaxAxesCmd           "QGATE_MAXAXES"
#define QG_CtrlDLLverCmd            "QGATE_DLLVER"
#define QG_CtrlSecurityCmd          "QGATE_SECURITY"
#define QG_CtrlReportCmd            "QGATE_REPORT"
#define QG_AxisNameCmd              "QGATE_NAMEAXIS"
#define QG_AxisModelCmd             "QGATE_STAGEMODEL"
#define QG_AxisConnectedCmd         "QGATE_AXISCONN"
#define QG_AxisModeCmd              "QGATE_AXISMODE"
#define QG_AxisSetPosCmd            "QGATE_POSITION"
#define QG_AxisPosCmd               "QGATE_POSITION_RBV"
#define QG_AxisMovingCmd            "QGATE_AXISMOVING"
#define QG_AxisInPosUnconfirmedCmd  "QGATE_INPOSU"
#define QG_AxisInPosLPFCmd          "QGATE_INPOSLPF"
#define QG_AxisInPosWindowCmd       "QGATE_INPOSWIN"

#define MAX_N_REPLIES (20)

//DLL's DoCommand Result list
class QGList : public std::list<std::string> {
public:
    QGList() {};
    std::string find(const int position);
    std::string print();
};

//Class for Queensgate controller
class QgateController : public asynMotorController {
    friend class QgateAxis;
public:
    enum {NOAXIS=-1};
public:
    QgateController(const char *portName, 
                    const char* serialPortName, 
                    const int maxNumAxes,
                    double movingPollPeriod, 
                    double idlePollPeriod,
                    const char* libraryPath);
    virtual ~QgateController();
    /* overridden methods */
    virtual asynStatus poll();
    virtual asynStatus setDeferredMoves(bool defer);

protected:
    // New parameters
    int QG_CtrlStatus;
    int QG_CtrlConnected;
    int QG_CtrlModel;
    int QG_CtrlFirmware;
    int QG_CtrlSerialNum;
    int QG_CtrlMaxStages;
    int QG_CtrlMaxAxes;
    int QG_CtrlDLLver;
    int QG_CtrlSecurity;
    int QG_CtrlReport;
    int QG_AxisName;
    int QG_AxisModel;
    int QG_AxisConnected;
    int QG_AxisMode;
    int QG_AxisPos; 
    int QG_AxisMoving;
    int QG_AxisInPosUnconfirmed;
    int QG_AxisInPosLPF;
    int QG_AxisInPosWindow;

protected:
    /* Methods for use by the axes */
    DllAdapter& getAdapter();
    bool isAxisPresent(int axisNum);
    DllAdapterStatus moveCmd(std::string cmd, int axisNum, double value);
    DllAdapterStatus getCmd(std::string cmd, int axisNum, std::string &value, int valueID=0);

private:
    asynUser* serialPortUser;
    DllAdapter qg;  //Queensgate adapter
    /* Config */
    std::string versionDLL;
    std::string model;
    std::string serialNum;
    std::string ctrl_firmware;
    std::string securityLevel;
    int numAxes;    //Configured amount of axes
    int maxAxes;    //Max amount of axes supported by the connected controller
    std::string portDevice;
protected:
    /* Status */
    std::string nameCtrl;
    bool initialised;   //Library comms successfully initialised
    bool connected;
    typedef std::vector<std::string> DeferredMoves;
    DeferredMoves deferredMove; //Stores the move commands to be deferred
    bool deferringMode;         //Moves are being deferred
private:
    asynStatus initController(const char* libPath);
    asynStatus initSession();
    asynStatus initialChecks();
    void printdefmoves();
};

#endif //QGATENPCcontroller_H_
