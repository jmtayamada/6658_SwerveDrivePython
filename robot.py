import wpilib as wp

from constants import *
from SwerveDrive import SwerveDrive


seconds = 0

class Robot(wp.TimedRobot):

    def __init__(self, period: seconds = 0.02):
        super().__init__(period)
        # create a swerveDrive by passing in SwerveModules, and keeping swervemodules in default positions
        self.SwerveDrive = SwerveDrive()
        # create joystick
        self.joystick = wp.Joystick(ControllerConstants.kDrivingJoystickPort)

    def teleopPeriodic(self):
        # run swerveDrive.drive function by passing in x axis, y axis, z axis form joystick, and the current angle
        self.SwerveDrive.drive(
            self.joystick.getRawAxis(0), 
            self.joystick.getRawAxis(1), 
            self.joystick.getRawAxis(2), 
        )


if __name__ == "__main__":
    wp.run(Robot)