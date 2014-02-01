
try:
    import wpilib
except ImportError:
    import fake_wpilib as wpilib



#pas import wpilib

lstick = wpilib.Joystick(1)

motor = wpilib.CANJaguar(8)
analog = wpilib.AnalogChannel(1)

class MotorOutput(wpilib.PIDOutput):
    def __init__(self, motor):
        super().__init__()
        self.motor = motor

    def PIDWrite(self, output):
        self.motor.Set(output)

class AnalogSource(wpilib.PIDSource):
    def __init__(self, analog):
        super().__init__()
        self.analog = analog

    def PIDGet(self):
        return analog.GetVoltage()

pidSource = AnalogSource(analog)
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
            pidController.SetSetpoint(2.5+lstick.GetY()*2.5)
           # print("in robot operator_control loop, pidOutput.motor.Get()=%d",pidOutput.motor.Get())

            wpilib.Wait(0.04)

def run():
    robot = MyRobot()
    robot.StartCompetition()
    
    return robot
