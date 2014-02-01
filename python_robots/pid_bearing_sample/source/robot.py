#see http://wpilib.screenstepslive.com/s/3120/m/7912/l/79828-operating-the-robot-with-feedback-from-sensors-pid-control
#Attempting to put the 'setpoint' = 0 degrees, then using the vision processing bearing control as the analog input.
#The pidcontroller should drive the motor till the analog input = the setpoint. 
#The vision code bearing result should call for changing direction until it reads zero, so we want to make the setpoint zero.
try:
    import wpilib
except ImportError:
    import fake_wpilib as wpilib

#from pynetworktables/sample/listener.py
from pynetworktables import *

ip = '127.0.0.1'

NetworkTable.SetIPAddress(ip)
NetworkTable.SetClientMode()
NetworkTable.Initialize()
class Listener(ITableListener):
    bearing = 0 #start at the 'no need to move' position
    def get_bearing(self):
      return self.bearing

    def __init__(self):
        ITableListener.__init__(self)
        
    def ValueChanged(self, table, key, value, isNew):
        if "Target Bearing:" == str(key):
          self.bearing = table.GetValue(key)

#        print("saw target bearing value")
#        print('Value changed: key %s, isNew: %s: %s' % (key, isNew, table.GetValue(key)))

listener = Listener()
        
table = NetworkTable.GetTable("SmartDashboard")
table.AddTableListener(listener)


#pas import wpilib

lstick = wpilib.Joystick(1)

motor  = wpilib.CANJaguar(8)
#analog = wpilib.AnalogChannel(1)

class MotorOutput(wpilib.PIDOutput):
    def __init__(self, motor):
        super().__init__()
        self.motor = motor

    def PIDWrite(self, output):
        self.motor.Set(output)

class VisionBearingSource(wpilib.PIDSource):
    def __init__(self, dashboard_listener):
        super().__init__()
        self.listener = dashboard_listener

    def PIDGet(self):
        return listener.get_bearing()
        #return analog.GetVoltage()

pidSource = VisionBearingSource(listener) #AnalogSource(analog)
pidOutput = MotorOutput(motor)
#pidController = wpilib.PIDController(1.0, 0.0, 0.0, pidSource, pidOutput)
pidController = wpilib.PIDController(0.1, 0.001, 0.0, pidSource, pidOutput)

def CheckRestart():
     if lstick.GetRawButton(10)[10]: #pas getrawbutton seems to be returning the entire button array and ignoring the button id param.
         raise RuntimeError("Restart") 

class MyRobot(wpilib.SimpleRobot):
    def Disabled(self):
        while self.IsDisabled():
            CheckRestart()
            wpilib.Wait(0.01)

    def Autonomous(self):
        self.GetWatchdog().SetEnabled(False)
        while self.IsAutonomous() and self.IsEnabled():
            CheckRestart()
            wpilib.Wait(0.01)

    def OperatorControl(self):
        dog = self.GetWatchdog()
        dog.SetEnabled(True)
        dog.SetExpiration(0.25)

        pidController.Enable()

        while self.IsOperatorControl() and self.IsEnabled():
            dog.Feed()
            CheckRestart()

            # Motor control
            #pidController.SetSetpoint(2.5+lstick.GetY()*2.5) #joystick output scaled to 0-5volts
            pidController.SetSetpoint(0) #aim for a bearing signal of zero
            print("bearing = %s" % listener.get_bearing())
           # print("in robot operator_control loop, pidOutput.motor.Get()=%d",pidOutput.motor.Get())

            wpilib.Wait(0.04)

def run():
    robot = MyRobot()
    robot.StartCompetition()
    
    return robot
