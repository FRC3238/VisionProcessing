from pynetworktables import *

# wpilib crashes if you don't do this.. 
SmartDashboard.init()

chooser = SendableChooser()
chooser.AddObject('choice 1', 'number1')
chooser.AddDefault('choice 2', 'number2')

SmartDashboard.PutData('choose', chooser)
SmartDashboard.PutString('what', 'who')
SmartDashboard.PutString('what', 'who')
SmartDashboard.PutString('what', 'who')
SmartDashboard.PutString('what', 'who')
SmartDashboard.PutString('what', 'who')
SmartDashboard.PutString('what', 'who')
SmartDashboard.PutString('what', 'who')
