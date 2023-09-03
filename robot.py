import wpilib as wp
import wpimath.kinematics
import wpimath.controller
import wpimath.geometry
import ctre
import math
import navx

seconds = 0

# Create a class for a swerveDriveMotorModule
class SwerveModule():

    def __init__(self, DriveMotorNum, TurnMotorNum, encoderChanA, encoderChanB) -> None:
        # Motor for moving
        self.DriveMotor = ctre.WPI_TalonSRX(DriveMotorNum)
        # motor for turning moving wheel
        self.TurnMotor = ctre.WPI_TalonSRX(TurnMotorNum)
        # Encoder for turning wheel
        self.TurnEncoder = wp.Encoder(encoderChanA, encoderChanB)
        self.TurnEncoder.setDistancePerPulse(360/4)                 # set distance per rotation to 360

        # create PID controller to turn the wheel
        self.TurnPID = wpimath.controller.PIDController(1, 0, 1)
        self.TurnPID.enableContinuousInput(0, 360)                  # enable continuous input on 0 to 360 to allow for optimised turning
        self.TurnPID.setTolerance(1)                                # set tolerance to one degree

        # Create the SwerveModuleState object that tracks what the motors should do
        self.moduleState = wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0))

    # function to get current angle in 0 to 360 degrees
    def getAngle(self):
        return self.TurnEncoder.getDistance() - (360 * math.floor(self.TurnEncoder.getDistance()/360))
    
    # function to run the motors
    def RunMotor(self):
        # set Drive motor speed to moduleState.speed parameter
        self.DriveMotor.set(self.moduleState.speed)
        # optimise swerveModuleState which combined with enabled continous input on PID controller makes it so that wheel will only turn at most 90 degrees
        self.moduleState.optimize(self.moduleState, wpimath.geometry.Rotation2d(self.getAngle()))
        # set Turn motor speed to PID.calculate of current angle, and modulestate.angle parameter
        self.TurnMotor.set(self.TurnPID.calculate(self.getAngle(), self.moduleState.angle.degrees()))


# Create a class for a swerve Drive
class SwerveDrive():

    # init with FL, FR, BL, and BR Swerve Modules, as well as the gyro. Swerve module positions can also be changed in the init
    def __init__(self, FL: SwerveModule, FR: SwerveModule, BL: SwerveModule, BR: SwerveModule, FLTranslation = wpimath.geometry.Translation2d(1, 1), FRTranslation = wpimath.geometry.Translation2d(-1, 1), BLTranslation = wpimath.geometry.Translation2d(1, -1), BRTranslation = wpimath.geometry.Translation2d(-1, -1)):
        # create the SwerveModules
        self.FL = FL
        self.FR = FR
        self.BL = BL
        self.BR = BR

        # add all of the swerve modules into a class to make them easier to use later
        self.swerveModules = [self.FL, self.FR, self.BL, self.BR]

        # Create the SwerveDriveKinematics with default modules placed evenly apart from the center. 
        # Can be changed when creating the swerve class
        self.swerveKinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            FLTranslation,
            FRTranslation,
            BLTranslation,
            BRTranslation
        )

    # Fucntion to move the swerve drive, input xaxis, yaxis, and zaxis of joystick, and a robot gyro angle for field oriented driving
    def drive(self, xAxis, yAxis, zAxis, gyroAngle = 0):
        # create chasisspeeds class from provided joystick inputs and current gyro position
        speeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
            xAxis,
            yAxis,
            zAxis,
            wpimath.geometry.Rotation2d(gyroAngle)
        )
        # feed chasisspeeds class into swerveKinematics.toSwerveModules, and set them to the swerveModuleStates
        self.FL.moduleState, self.FR.moduleState, self.BL.moduleState, self.BR.moduleState = \
            self.swerveKinematics.toSwerveModuleStates(speeds)
        # desaturate the wheel speeds to prevent errors (setting motor power higher than 1)
        self.swerveKinematics.desaturateWheelSpeeds(
            tuple(
                [self.FL.moduleState, self.FR.moduleState, self.BL.moduleState, self.BR.moduleState]
            ), 1
        )
        # for each swerveModule, run the RunMotor function
        for obj in self.swerveModules:
            obj.RunMotor()


class Robot(wp.TimedRobot):

    def __init__(self, period: seconds = 0.02):
        super().__init__(period)
        # Create the SwerveModule classes
        self.FLSwerveMod = SwerveModule(0, 1, 0, 1)
        self.FRSwerveMod = SwerveModule(2, 3, 2, 3)
        self.BLSwerveMod = SwerveModule(4, 5, 4, 5)
        self.BRSwerveMod = SwerveModule(6, 7, 6, 7)
        # Create a navx gyro
        self.gyro = navx.AHRS(wp.SPI.Port(0), 1)
        # create a swerveDrive by passing in SwerveModules, and keeping swervemodules in default positions
        self.SwerveDrive = SwerveDrive(self.FLSwerveMod, self.FRSwerveMod, self.BLSwerveMod, self.BRSwerveMod)
        # create joystick
        self.joystick = wp.Joystick(0)

    def teleopPeriodic(self):
        # run swerveDrive.drive function by passing in x axis, y axis, z axis form joystick, and the current angle
        self.SwerveDrive.drive(
            self.joystick.getRawAxis(0), 
            self.joystick.getRawAxis(1), 
            self.joystick.getRawAxis(2), 
            gyroAngle=self.gyro.getAngle()
        )


if __name__ == "__main__":
    wp.run(Robot)