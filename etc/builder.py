from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.motor import MotorLib
from iocbuilder.modules.asyn import Asyn, AsynIP, AsynPort
from iocbuilder.arginfo import makeArgInfo, Simple, Ident, Choice, Enum
import os.path

__all__ = ['NPCcontroller', 'NPCaxis', 'NScontroller', 'NSsensor']

class _QueensgateNPCpars(AutoSubstitution):
    TemplateFile = "NPCcontroller.template"

class _QueensgateNPCaxisPars(AutoSubstitution):
    TemplateFile = "NPCaxis.template"

class _QueensgateNSpars(AutoSubstitution):
    TemplateFile = "NScontroller.template"

class _QueensgateNSsensorPars(AutoSubstitution):
    TemplateFile = "NSsensor.template"

def _defaultPathLib(self):
    if self.libPath=="":
        self.libPath = os.path.abspath(os.path.dirname(__file__) + "/../lib")
        self.libPath = os.path.join(self.libPath, "linux-x86_64/controller_interface64.so")

class NPCcontroller(Device):
    '''NPC-series Queensgate NanoPositioner Serial controller'''
    Dependencies = (Asyn, MotorLib) #, Asynaid)
    LibFileList = ['queensgateNPC']
    DbdFileList = ['queensgateNPC', 'systemCommandSupport']

    def __init__(self, name, P, Q, numAxes, portAddress, libPath="", fastPoll=1, slowPoll=5, virtual_port="", virtual_wait=1.0):
        self.__dict__.update(locals())
        self.template = _QueensgateNPCpars(PORT=name, P=P, Q=Q, TIMEOUT=5, name=name)
        self.__super.__init__()

    ArgInfo = makeArgInfo(__init__,
                        name=Simple("GUI name", str),
                        P=Simple("Device Prefix", str),
                        Q=Simple("Device Suffix", str),
                        #XXX: asynPort=Ident("Asyn port name", AsynPort),
                        numAxes=Simple("Amount of axes configured", int),
                        portAddress=Simple("Low level Address of the physical port (usually /dev/ttyX)", str),
                        fastPoll=Simple("Fast polling rate, in seconds (e.g. 0.5)", float),
                        slowPoll=Simple("Slow polling rate, in seconds (e.g. 1.7)", float),
                        libPath=Simple("Path and name for the Queensgate SDK library -- leave empty for using the one coming with the support module", str),
                        virtual_port=Simple("IP address and port to use socat for connection (e.g. 123.134.145.156:7009)", str),
                        virtual_wait=Simple("Time (in secs) to wait after setting up the virtual port", float),
                        )

    def Initialise(self):
        _defaultPathLib(self)
        #initialise socat if virtual port present
        if not self.virtual_port=="":
            #Create our socat command to create virtual port to specified address
            virtual_port_string = ('system "socat pty,link={serial_port},raw tcp:{ip_address}&"')
            print('# Create virtual port using socat to connect to device')
            print(virtual_port_string.format(
                    serial_port=self.portAddress,
                    ip_address=self.virtual_port
                    )
                )
            # print('system "ps aux | grep socat"')
            # In practice a delay is needed as the command is run in the background
            # that should give enough time to clean up an old port from killing
            # an old IOC process and set it up ready for the new connection.
            print('# Sleep for {virtual_wait} seconds to give socat time to prepare'.format(**self.__dict__))
            print('epicsThreadSleep {virtual_wait}'.format(**self.__dict__))
        print('qgateCtrlConfig( "{name}", "{portAddress}", "{numAxes}", "{fastPoll}", "{slowPoll}", "{libPath}" )'.format(**self.__dict__))
        # print('# done IOCing')

class NPCaxis(Device):
    '''Axis (stage) in NPC-series Queensgate NanoPositioner controller'''
    def __init__(self, name, controller, P, Q, axis, axisMode=0, timeout=5, DIR="Pos", MRES=.000001, DHLM=10, DLLM=-10, PREC=6, EGU="um"):
        self.__dict__.update(locals())
        #TODO: check axis num [1..n]
        self.axisIndex = axis - 1
        self.template = _QueensgateNPCaxisPars(PORT=controller.name, P=P, Q=Q, AXIS=self.axisIndex, TIMEOUT=timeout, name=name, 
                        dir=DIR, mres=MRES, dhlm=DHLM, dllm=DLLM, prec=PREC, egu=EGU)
        self.__super.__init__()
    axisModeChoice = ["Native","Unconfirmed","Window-confirmed","LPF-Confirmed","Window-and-LPF-confirmed"]
    ArgInfo = makeArgInfo(__init__,
                          name=Simple("Axis name", str),
                          controller = Ident("controller port name", NPCcontroller.__name__),
                          P=Simple("Device Prefix", str),
                          Q=Simple("Device Suffix", str),
                          axis=Simple("Stage number connected to the controller [1..n]", int),
                          axisMode=Choice("Moving indication mode",range(0, 5), axisModeChoice),
                          timeout=Simple("Comms timeout", str),
                          DIR=Simple("Motor Direction (Pos/Neg)", str),
                          MRES=Simple("Motion resolution (automatically applies same to ERES and RRES)", float), 
                          DHLM=Simple("Dial high limit", float), 
                          DLLM=Simple("Dial low limit", float), 
                          PREC=Simple("Display precision", float), 
                          EGU=Simple("Engineering units", float)
                          )

    def Initialise(self):
        print('qgateAxisConfig( "{controller.name}", "{axis}", "{name}", {axisMode} )'.format(**self.__dict__))

class NScontroller(Device):
    '''NS-series Queensgate Serial NanoSensor controller'''
    Dependencies = (Asyn, MotorLib) #, Asynaid)
    LibFileList = ['queensgateNPC']
    DbdFileList = ['queensgateNPC', 'systemCommandSupport']

    def __init__(self, name, P, Q, numAxes, portAddress, libPath="", fastPoll=1, slowPoll=5, virtual_port="", virtual_wait=1.0):
        self.__dict__.update(locals())
        self.template = _QueensgateNSpars(PORT=name, P=P, Q=Q, TIMEOUT=5, name=name)
        self.__super.__init__()

    ArgInfo = makeArgInfo(__init__,
                        name=Simple("GUI name", str),
                        P=Simple("Device Prefix", str),
                        Q=Simple("Device Suffix", str),
                        numAxes=Simple("Amount of channels configured", int),
                        portAddress=Simple("Low level Address of the physical port (usually /dev/ttyX)", str),
                        fastPoll=Simple("Fast polling rate, in seconds (e.g. 0.5)", float),
                        slowPoll=Simple("Slow polling rate, in seconds (e.g. 1.7)", float),
                        libPath=Simple("Path and name for the Queensgate SDK library -- leave empty for using the one coming with the support module", str),
                        virtual_port=Simple("IP address and port to use socat for connection (e.g. 123.134.145.156:7009)", str),
                        virtual_wait=Simple("Time (in secs) to wait after setting up the virtual port", float),
                        )

    def Initialise(self):
        _defaultPathLib(self)
        #initialise socat if virtual port present
        if not self.virtual_port=="":
            #Create our socat command to create virtual port to specified address
            virtual_port_string = ('system "socat pty,link={serial_port},raw tcp:{ip_address}&"')
            print('# Create virtual port using socat to connect to device')
            print(virtual_port_string.format(
                    serial_port=self.portAddress,
                    ip_address=self.virtual_port
                    )
                )
            # print('system "ps aux | grep socat"')
            # In practice a delay is needed as the command is run in the background
            # that should give enough time to clean up an old port from killing
            # an old IOC process and set it up ready for the new connection.
            print('# Sleep for {virtual_wait} seconds to give socat time to prepare'.format(**self.__dict__))
            print('epicsThreadSleep {virtual_wait}'.format(**self.__dict__))
        print('qgateCtrlConfig( "{name}", "{portAddress}", "{numAxes}", "{fastPoll}", "{slowPoll}", "{libPath}" )'.format(**self.__dict__))
        # print('# done IOCing')
        
class NSsensor(Device):
    '''Channel (stage) in Queensgate NS-series NanoSensor controller'''
    def __init__(self, name, controller, P, Q, axis, axisMode=3, timeout=5, DIR="Pos", MRES=.000001, DHLM=1000, DLLM=-1000, PREC=6, EGU="um"):
        self.__dict__.update(locals())
        self.axisIndex = axis - 1
        self.template = _QueensgateNSsensorPars(PORT=controller.name, P=P, Q=Q, AXIS=self.axisIndex, TIMEOUT=timeout, name=name, 
                        dir=DIR, mres=MRES, dhlm=DHLM, dllm=DLLM, prec=PREC, egu=EGU)
        self.__super.__init__()
    axisModeChoice = ["Native","Unconfirmed","Window-confirmed","LPF-Confirmed","Window-and-LPF-confirmed"]
    ArgInfo = makeArgInfo(__init__,
                          name=Simple("Axis name", str),
                          controller = Ident("controller port name", NPCcontroller.__name__),
                          P=Simple("Device Prefix", str),
                          Q=Simple("Device Suffix", str),
                          axis=Simple("Stage number connected to the controller [1..n]", int),
                          axisMode=Choice("Moving indication mode",range(0, 5), axisModeChoice),
                          timeout=Simple("Comms timeout", str),
                          DIR=Simple("Motor Direction (Pos/Neg)", str),
                          MRES=Simple("Motion resolution (automatically applies same to ERES and RRES)", float), 
                          DHLM=Simple("Dial high limit", float), 
                          DLLM=Simple("Dial low limit", float), 
                          PREC=Simple("Display precision", float), 
                          EGU=Simple("Engineering units", str)
                          )

    def Initialise(self):
        # Note that axis type 1 is a sensor
        print('qgateAxisConfig( "{controller.name}", "{axis}", "{name}", {axisMode}, 1 )'.format(**self.__dict__))
        

