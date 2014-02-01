try:
    import wpilib
except ImportError:
    import fake_wpilib as wpilib

def CheckRestart():
     wpilib.Wait(0.01)
     # if lstick.GetRawButton(10)[10]: #pas getrawbutton seems to be returning the entire button array and ignoring the button id param.
     #     raise RuntimeError("Restart") 

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

        while self.IsOperatorControl() and self.IsEnabled():
            dog.Feed()
            CheckRestart()
            wpilib.SmartDashboard.PutString("python_robot_says", "I'm not dead yet.")
            print("python_robot says, I'm not dead yet, to the console")

            wpilib.Wait(0.04)

def run():
    robot = MyRobot()
    robot.StartCompetition()
    
    return robot
