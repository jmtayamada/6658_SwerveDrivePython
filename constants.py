import rev

from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile
from wpimath import units

import math


#  * The Constants class provides a convenient place for teams to hold robot-wide
#  * numerical or boolean
#  * constants. This class should not be used for any other purpose. All constants
#  * should be declared
#  * globally (i.e. public static). Do not put anything functional in this class.
#  *
#  * <p>
#  * It is advised to statically import this class (or one of its inner classes)
#  * wherever the
#  * constants are needed, to reduce verbosity.

class ControllerConstants:
    kDrivingJoystickPort = 0
    kHelperJoystickPort = 1

# class NeoMotorConstants:
#     kFreeSpeedRpm = 5676
#     # make sure to also change variable in line 69

class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    # kMaxSpeedMetersPerSecond = 4.8

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(26.5)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(26.5)
    # Distance between front and back wheels on robot
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2))

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 11
    kRearLeftDrivingCanId = 13
    kFrontRightDrivingCanId = 15
    kRearRightDrivingCanId = 17

    kFrontLeftTurningCanId = 10
    kRearLeftTurningCanId = 12
    kFrontRightTurningCanId = 14
    kRearRightTurningCanId = 16

    kGyroReversed = False


class ModuleConstants:
    # kFreeSpeedRpm = 5676

    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 14

    # Invert the turning encoder, since the output shaft rotates in the opposite direction of
    # the steering motor in the MAXSwerve Module.
    kTurningEncoderInverted = True

    # Calculations required for driving motor conversion factors and feed forward
    # kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60
    kWheelDiameterMeters = .1016 # 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    # kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction # meters
    kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) / 60.0 # meters per second

    kTurningEncoderPositionFactor = (2 * math.pi) # radians
    kTurningEncoderVelocityFactor = (2 * math.pi) / 60.0 # radians per second

    kTurningEncoderPositionPIDMinInput = 0 # radians
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor # radians

    # kDrivingP = 0.04
    # kDrivingI = 0
    # kDrivingD = 0
    # kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    # kDrivingMinOutput = -1
    # kDrivingMaxOutput = 1

    kTurningP = 1
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = rev.CANSparkMax.IdleMode.kBrake
    kTurningMotorIdleMode = rev.CANSparkMax.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50 # amps
    kTurningMotorCurrentLimit = 20 # amps
