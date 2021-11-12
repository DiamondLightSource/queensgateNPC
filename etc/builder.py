from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.motor import MotorLib
# from iocbuilder.modules.asynaid import Asynaid
from iocbuilder.modules.asyn import Asyn, AsynIP, AsynPort
from iocbuilder.arginfo import makeArgInfo, Simple, Ident

__all__ = ['NPCcontroller', 'NPCaxis']

class _QueensgateNPCpars(AutoSubstitution):
    TemplateFile = "NPCcontroller.template"

class _QueensgateNPCaxisPars(AutoSubstitution):
    TemplateFile = "NPCaxis.template"

class NPCcontroller(Device):
    '''NPC-series Queensgate Serial controller'''
    Dependencies = (Asyn, MotorLib) #, Asynaid)
    LibFileList = ['queensgateNPC']
    DbdFileList = ['queensgateNPC', 'systemCommandSupport']

    def __init__(self, name, P, Q, numAxes, portAddress, libPath, fastPoll=1, slowPoll=5, virtual_port=""):
        self.__dict__.update(locals())
        #XXX: self.template = _QueensgateNPCpars(PORT='{P}{Q}_AP'.format(P=P, Q=Q), P=P, Q=Q, TIMEOUT=5, name=name)
        self.template = _QueensgateNPCpars(PORT=name, P=P, Q=Q, TIMEOUT=5, name=name)
        self.__super.__init__()

    ArgInfo = makeArgInfo(__init__,
                          name=Simple("GUI name", str),
                          P=Simple("Device Prefix", str),
                          Q=Simple("Device Suffix", str),
                          #XXX: asynPort=Simple("Asyn port name", str),
                          #XXX: asynPort=Ident("Asyn port name", AsynPort),
                          numAxes=Simple("Amount of axes configured", int),
                          portAddress=Simple("Low level Address of the physical port (usually /dev/ttyX)", str),
                          fastPoll=Simple("Fast polling rate, in seconds (e.g. 0.5)", float),
                          slowPoll=Simple("Slow polling rate, in seconds (e.g. 1.7)", float),
                          libPath=Simple("Path and name for the Queensgate SDK library", str),
			  virtual_port=Simple("IP address and port to use socat for connection (e.g. 123.134.145.156:7009)", str)
                          )

    def Initialise(self):
        #initialise socat if virtual port present
        if not self.virtual_port=="":
            #NPCcontroller.DbdFileList += ['systemCommandSupport']
            print("DBDs: ", NPCcontroller.DbdFileList)
            #Create our socat command to create virtual port to specified address
            virtual_port_string = ('system "socat pty,link={serial_port},raw '
                                'tcp:{ip_address}&"')
            print('# Create virtual port using socat to connect to device')
            print(virtual_port_string.format(
                    serial_port=self.portAddress,
                    ip_address=self.virtual_port
                    )
                )
            print("# SHOULD BE socat -d -d -x pty,link=/tmp/qgate1,raw,echo=0 tcp:172.23.112.6:7017")
            print('system "ps aux | grep socat"')
            # In practice a delay is needed as the command is run in the background.
            # 5 seconds should give enough time to clean up an old port from killing
            # an old IOC process and set it up ready for the new connection.
            print('# Sleep for 5 seconds to give socat time to prepare')
            print('epicsThreadSleep 5')
        #XXX:print('qgateCtrlConfig "{P}_AP", "{serial}", "{logpath}"'.format(**self.__dict__))
        #XXX:print('qgateCtrlConfig "{P}{Q}_AP", "{asynPort}", "{portAlias}", "{libPath}"'.format(**self.__dict__))
        #XXX: print('qgateCtrlConfig(' \
        #     ' "%(name)s",' \
        #     ' "%(asynPort)s",' \
        #     ' "%(portAlias)s",' \
        #     ' "%(libPath)s" )' ) \
        #     % self.__dict__
        #XXX: print('qgateCtrlConfig( "{name}", "{asynPort}", "{asynPortAlias}", "{libPath}" )'.format(**self.__dict__))
        # print('qgateCtrlConfig( "{name}", "{asynPort}", "{portAlias}", "{libPath}" )'.format(**self.__dict__))
        print('qgateCtrlConfig( "{name}", "{portAddress}", "{numAxes}", "{fastPoll}", "{slowPoll}", "{libPath}" )'.format(**self.__dict__))

class NPCaxis(Device):
    '''Axis (stage) in Queensgate controller'''
    #Dependencies = (Asyn,)
    #LibFileList = ['queensgateNPC']
    #DbdFileList = ['queensgateNPC']
    #TemplateFile = 'NPCaxis.template'
    # axisIndex = 0
    #XXX:def __init__(self, name, P, Q, asynPort, channel, timeout=5):
    def __init__(self, name, controller, P, Q, axis, timeout=5, DIR="Pos", MRES=.000001, DHLM=10, DLLM=-10, PREC=6, EGU="um"):
        self.__dict__.update(locals())
        #TODO: check axis num [1..n]
        #XXX: self.template = _QueensgateNPCaxisPars(PORT='{P}{Q}_AP'.format(P=P, Q=Q), P=P, Q=Q, AXIS=axis, TIMEOUT=timeout, name=name)
        self.axisIndex = axis - 1
        # self.template = _QueensgateNPCaxisPars(PORT=controller.name, P=P, Q=Q, AXIS=axis, TIMEOUT=timeout, name=name)
        self.template = _QueensgateNPCaxisPars(PORT=controller.name, P=P, Q=Q, AXIS=self.axisIndex, TIMEOUT=timeout, name=name, 
                        dir=DIR, mres=MRES, dhlm=DHLM, dllm=DLLM, prec=PREC, egu=EGU)
        self.__super.__init__()
    ArgInfo = makeArgInfo(__init__,
                          name=Simple("Axis name", str),
                          #asynPort=Simple("Asyn port name", str),
                          controller = Ident("controller port name", NPCcontroller.__name__),
                          P=Simple("Device Prefix", str),
                          Q=Simple("Device Suffix", str),
                          #channel=Simple("Stage number connected to the controller", str),
                          axis=Simple("Stage number connected to the controller [1..n]", int),
                          timeout=Simple("Comms timeout", str),
                          DIR=Simple("Motor Direction (Pos/Neg)", str),
                          MRES=Simple("Motion resolution (automatically applies same to ERES and RRES)", float), 
                          DHLM=Simple("Dial high limit", float), 
                          DLLM=Simple("Dial low limit", float), 
                          PREC=Simple("Display precision", float), 
                          EGU=Simple("Engineering units", float)
                          )

    def Initialise(self):
        #XXX:print('# Queensgate NPC init')
        #XXX:print('qgateAxisConfig "{P}_AP", "{serial}", "{logpath}"'.format(**self.__dict__))
        #XXX: print('qgateAxisConfig("%(controller)s", %(axis)d, "%(name)s")'.format(**self.__dict__))
        #XXX: print 'qgateAxisConfig("%(PORT)s", %(axis)d, "%(name)s")' % self.__dict__
        print('qgateAxisConfig( "{controller.name}", "{axis}", "{name}" )'.format(**self.__dict__))
        #print('qgateAxisConfig( "{controller.name}", "{self.axisIndex}", "{name}" )'.format(**self.__dict__))
        
    
