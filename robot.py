import wpilib as wp

from constants import *
from SwerveDrive import SwerveDrive

from photonlibpy.photonCamera import *
from photonlibpy.multiTargetPNPResult import *
from photonlibpy.photonPipelineResult import *
from photonlibpy.photonTrackedTarget import *

from SwerveModule import SwerveModule
from wpilib import SmartDashboard

seconds = 0

class Robot(wp.TimedRobot):

    def __init__(self, period: seconds = 0.02):
        super().__init__(period)
        self.SwerveDrive = SwerveDrive()
        # create joystick
        self.driverStick = wp.Joystick(ControllerConstants.kDrivingJoystickPort)
        self.helperStick = wp.Joystick(ControllerConstants.kHelperJoystickPort)

        self.testSwerveModule = SwerveModule(0, 1, 1)

    def teleopPeriodic(self):
        # run swerveDrive.drive function by passing in x axis, y axis, z axis form joystick
        self.SwerveDrive.drive(
            self.driverStick.getRawAxis(0), 
            self.driverStick.getRawAxis(1), 
            self.driverStick.getRawAxis(2), 
        )

    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self) -> None:
        x = self.driverStick.getRawAxis(0)
        y = self.driverStick.getRawAxis(1)
        speed = math.sqrt(x**2+y**2)
        angle = math.atan2(y, x)
        SmartDashboard.putNumber("Current Angle", self.testSwerveModule.getState().angle)
        SmartDashboard.putNumber("Current Position", self.testSwerveModule.getState().speed)
        self.testSwerveModule.testMotors(speed, angle)


if __name__ == "__main__":
    wp.run(Robot)