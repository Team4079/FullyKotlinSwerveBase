package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.utils.GlobalsValues.MotorGlobalValues
import frc.robot.utils.GlobalsValues.SwerveGlobalValues
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal

/** The [SwerveModule] class includes all the motors to control the swerve drive. */
class SwerveModule(
  driveId: Int,
  steerId: Int,
  canCoderID: Int,
  canCoderDriveStraightSteerSetPoint: Double,
) {
  /** Drive motor for the swerve module */
  private val driveMotor: TalonFX = TalonFX(driveId)

  /** CANcoder for the swerve module */
  private val canCoder: CANcoder = CANcoder(canCoderID)

  /** Steer motor for the swerve module */
  private val steerMotor: TalonFX = TalonFX(steerId)

  /** Position voltage control for the swerve module */
  private val positionSetter: PositionVoltage =
    PositionVoltage(0.0, 0.0, true, 0.0, 0, true, false, false).withSlot(0)

  /** Velocity torque current FOC control for the swerve module */
  private val velocitySetter: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)

  /** Position of the swerve module */
  private val swerveModulePosition: SwerveModulePosition = SwerveModulePosition()

  /** State of the swerve module */
  var state: SwerveModuleState = SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0))
    get() {
      field.angle = Rotation2d.fromRotations(steerMotor.position.valueAsDouble)
      field.speedMetersPerSecond =
        (driveMotor.velocity.valueAsDouble *
          (MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO / MotorGlobalValues.METERS_PER_REVOLUTION))
      return field
    }
    set(value) {
      val newPosition = position
      val optimized = SwerveModuleState.optimize(value, newPosition.angle)

      val angleToSet = optimized.angle.rotations
      SmartDashboard.putNumber(
        "desired state after optimize " + canCoder.deviceID,
        optimized.angle.rotations,
      )

      steerMotor.setControl(positionSetter.withPosition(angleToSet))

      val velocityToSet =
        (optimized.speedMetersPerSecond *
          (MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO / MotorGlobalValues.METERS_PER_REVOLUTION))
      driveMotor.setControl(velocitySetter.withVelocity(velocityToSet))

      field = value
    }

  /** Position of the swerve module */
  val position: SwerveModulePosition
    get() {
      driveVelocity = driveMotor.velocity.valueAsDouble
      drivePosition = driveMotor.position.valueAsDouble
      steerVelocity = steerMotor.velocity.valueAsDouble
      steerPosition = steerMotor.position.valueAsDouble

      swerveModulePosition.angle = Rotation2d.fromRotations(steerPosition)
      swerveModulePosition.distanceMeters =
        (drivePosition /
          (MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO / MotorGlobalValues.METERS_PER_REVOLUTION))

      return swerveModulePosition
    }

  /** Drive velocity of the swerve module */
  private var driveVelocity: Double

  /** Drive position of the swerve module */
  private var drivePosition: Double

  /** Steer position of the swerve module */
  private var steerPosition: Double

  /** Steer velocity of the swerve module */
  private var steerVelocity: Double

  init {
    state = SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0))

    /** Configuration for the drive motor */
    val driveConfigs =
      TalonFXConfiguration().apply {
        Slot0.apply {
          kP = BasePIDGlobal.DRIVE_PID.p
          kI = BasePIDGlobal.DRIVE_PID.i
          kD = BasePIDGlobal.DRIVE_PID.d
        }
        MotorOutput.apply {
          NeutralMode = NeutralModeValue.Brake
          Inverted = SwerveGlobalValues.DRIVE_MOTOR_INVERETED
        }
      }

    /** Configuration for the steer motor */
    val steerConfigs =
      TalonFXConfiguration().apply {
        Slot0.apply {
          kP = BasePIDGlobal.STEER_PID.p
          kI = BasePIDGlobal.STEER_PID.i
          kD = BasePIDGlobal.STEER_PID.d
        }
        MotorOutput.apply {
          NeutralMode = NeutralModeValue.Brake
          Inverted = SwerveGlobalValues.STEER_MOTOR_INVERTED
        }
        Feedback.apply {
          FeedbackRemoteSensorID = canCoderID
          FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
          RotorToSensorRatio = MotorGlobalValues.STEER_MOTOR_GEAR_RATIO
        }
        ClosedLoopGeneral.ContinuousWrap = true
      }

    /** Configuration for the CANcoder */
    val canCoderConfiguration =
      CANcoderConfiguration().MagnetSensor.apply {
        AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf
        SensorDirection = SensorDirectionValue.CounterClockwise_Positive
        MagnetOffset = SwerveGlobalValues.ENCODER_OFFSET + canCoderDriveStraightSteerSetPoint
      }

    /** Apply configurations to the drive motor */
    driveMotor.apply {
      configurator.apply(driveConfigs)
      driveVelocity = velocity.valueAsDouble
      drivePosition = position.valueAsDouble
    }

    /** Apply configurations to the steer motor */
    steerMotor.apply {
      configurator.apply(steerConfigs)
      steerVelocity = velocity.valueAsDouble
      steerPosition = position.valueAsDouble
    }

    /** Apply configurations to the CANcoder */
    canCoder.configurator.apply(canCoderConfiguration)
  }
}
