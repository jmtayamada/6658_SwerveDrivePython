from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from rev import CANSparkMax, SparkMaxAbsoluteEncoder
import rev

from constants import *

class SwerveModule():
    def __init__(self, DriveMotorNum, TurnMotorNum) -> None:
        # Create objects for driving and turning motor, then rest settings
        self.DriveMotor = CANSparkMax(DriveMotorNum, rev._rev.CANSparkMax.MotorType.kBrushless)
        self.DriveMotor.restoreFactoryDefaults()
        self.TurnMotor = CANSparkMax(TurnMotorNum, rev._rev.CANSparkMax.MotorType.kBrushless)
        self.TurnMotor.restoreFactoryDefaults()
        # Create encoders
        self.DriveEncoder = self.DriveMotor.getEncoder()
        self.TurnEncoder = self.TurnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

        # create PID controllers
        self.TurnPIDController = self.TurnMotor.getPIDController()
        self.TurnPIDController.setFeedbackDevice(self.TurnEncoder)
        self.DrivePIDController = self.DriveMotor.getPIDController()
        self.DrivePIDController.setFeedbackDevice(self.DriveEncoder)

        # Apply position and velocity conversion factors for the driving encoder. The
        # native units for position and velocity are rotations and RPM, respectively,
        # but we want meters and meters per second to use with WPILib's swerve APIs.
        self.DriveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        self.DriveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)

        # Apply position and velocity conversion factors for the turning encoder. We
        # want these in radians and radians per second to use with WPILib's swerve APIs.
        self.TurnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        self.TurnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)

        self.TurnEncoder.setInverted(ModuleConstants.kTurningEncoderInverted)

        # PID settings
        self.TurnPIDController.setPositionPIDWrappingEnabled(True)
        self.TurnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)
        self.TurnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)

        # self.DrivePIDController.setP(ModuleConstants.kDrivingP)
        # self.DrivePIDController.setI(ModuleConstants.kDrivingI)
        # self.DrivePIDController.setD(ModuleConstants.kDrivingD)
        # self.DrivePIDController.setFF(ModuleConstants.kDrivingFF)
        # self.DrivePIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        #     ModuleConstants.kDrivingMaxOutput)
        
        self.TurnPIDController.setP(ModuleConstants.kTurningP)
        self.TurnPIDController.setI(ModuleConstants.kTurningI)
        self.TurnPIDController.setD(ModuleConstants.kTurningD)
        self.TurnPIDController.setFF(ModuleConstants.kTurningFF)
        self.TurnPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
            ModuleConstants.kTurningMaxOutput)
        
        self.DriveMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode)
        self.TurnMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode)
        self.DriveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
        self.TurnMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)

        # Save the SPARK MAX configurations. If a SPARK MAX browns out during
        # operation, it will maintain the above configurations.
        self.DriveMotor.burnFlash()
        self.TurnMotor.burnFlash()

        self.DriveEncoder.setPosition(0)

    # get current SwerveModuleState
    def getState(self):
        return SwerveModuleState(self.DriveEncoder.getVelocity(), Rotation2d(self.TurnEncoder.getPosition()))

    # get current SwerveModule Position
    def getPosition(self):
        return SwerveModulePosition(self.DriveEncoder.getPosition(), Rotation2d(self.TurnEncoder.getPosition()))
    
    # function to run the motors
    def setDesiredState(self, desiredState: SwerveModuleState):
        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = SwerveModuleState.optimize(desiredState, Rotation2d(self.TurnEncoder.getPosition()))

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.DriveMotor.set(optimizedDesiredState.speed)
        # self.DrivePIDController.setReference(optimizedDesiredState.speed, CANSparkMax.ControlType.kVelocity)
        self.TurnPIDController.setReference(optimizedDesiredState.angle.radians(), CANSparkMax.ControlType.kPosition)
    
    # rest encoders
    def resetEncoders(self):
        self.DriveEncoder.setPosition(0)