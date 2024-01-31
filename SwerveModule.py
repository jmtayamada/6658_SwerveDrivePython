from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
from rev import CANSparkMax
from phoenix5.sensors import CANCoder

from constants import *

class SwerveModule():
    def __init__(self, DriveMotorNum, TurnMotorNum, EncoderNum) -> None:
        # Create objects for driving and turning motor, then rest settings
        self.DriveMotor = CANSparkMax(DriveMotorNum, CANSparkMax.MotorType.kBrushless)
        self.DriveMotor.restoreFactoryDefaults()
        self.TurnMotor = CANSparkMax(TurnMotorNum, CANSparkMax.MotorType.kBrushless)
        self.TurnMotor.restoreFactoryDefaults()
        # Create encoders
        self.DriveEncoder = self.DriveMotor.getEncoder()
        self.TurnEncoder = CANCoder(EncoderNum)
        # self.TurnEncoder = CANCoder(EncoderNum)

        # create PID controllers
        self.TurnPIDController = PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
        self.DrivePIDController = self.DriveMotor.getPIDController()
        self.DrivePIDController.setFeedbackDevice(self.DriveEncoder)

        # Apply position and velocity conversion factors for the driving encoder. The
        # native units for position and velocity are rotations and RPM, respectively,
        # but we want meters and meters per second to use with WPILib's swerve APIs.
        self.DriveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        self.DriveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)

        # PID settings
        self.TurnPIDController.enableContinuousInput(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput)

        self.DrivePIDController.setP(ModuleConstants.kDrivingP)
        self.DrivePIDController.setI(ModuleConstants.kDrivingI)
        self.DrivePIDController.setD(ModuleConstants.kDrivingD)
        self.DrivePIDController.setFF(ModuleConstants.kDrivingFF)
        self.DrivePIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
            ModuleConstants.kDrivingMaxOutput)
        
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
        return SwerveModuleState(self.DriveEncoder.getVelocity(), Rotation2d(self.TurnEncoder.getAbsolutePosition()))

    # get current SwerveModule Position
    def getPosition(self):
        return SwerveModulePosition(self.DriveEncoder.getPosition(), Rotation2d(self.TurnEncoder.getAbsolutePosition()))
    
    # function to run the motors
    def setDesiredState(self, desiredState: SwerveModuleState):
        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = SwerveModuleState.optimize(desiredState, Rotation2d(self.TurnEncoder.getAbsolutePosition()*math.pi/180))

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.DrivePIDController.setReference(optimizedDesiredState.speed, CANSparkMax.ControlType.kVelocity)
        self.TurnMotor.set(self.TurnPIDController.calculate(self.TurnEncoder.getAbsolutePosition(), optimizedDesiredState.angle.degrees()))
        # self.TurnPIDController.setReference(optimizedDesiredState.angle.radians(), CANSparkMax.ControlType.kPosition)
    
    def testMotors(self, velocity, angle):
        self.DrivePIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity)
        self.TurnMotor.set(self.TurnPIDController.calculate(self.TurnEncoder.getAbsolutePosition(), angle))
    
    # rest encoders
    def resetEncoders(self):
        self.DriveEncoder.setPosition(0)