from SwerveModule import SwerveModule
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState
from navx import AHRS
from constants import DriveConstants
from wpilib import SPI

class SwerveDrive():
    def __init__(self) -> None:

        self.FLSwerve = SwerveModule(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftEncoderCanId)
        self.FRSwerve = SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightEncoderCanId)
        self.RLSwerve = SwerveModule(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId, DriveConstants.kRearLeftEncoderCanId)
        self.RRSwerve = SwerveModule(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId, DriveConstants.kRearRightEncoderCanId)

        self.gyro = AHRS(SPI.Port(0))

    def getHeading(self):
        return Rotation2d.fromDegrees(self.gyro.getAngle())
    
    def getTurnRate(self):
        return self.gyro.getRate() * ((float(DriveConstants.kGyroReversed) - 0.5)/.5)
    
    def zeroHeading(self):
        self.gyro.reset()

    def resetEncoders(self):
        self.FLSwerve.resetEncoders()
        self.FRSwerve.resetEncoders()
        self.RLSwerve.resetEncoders()
        self.RRSwerve.resetEncoders()

    def setModuleStates(self, desiredStates: list):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.FLSwerve.setDesiredState(desiredStates[0])
        self.FRSwerve.setDesiredState(desiredStates[1])
        self.RLSwerve.setDesiredState(desiredStates[2])
        self.RRSwerve.setDesiredState(desiredStates[3]) 

    def setX(self):
        self.FLSwerve.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.FRSwerve.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.RLSwerve.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.RRSwerve.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def drive(self, xSpeed: float, ySpeed: float, rot: float):
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(self.gyro.getAngle()))
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)
        