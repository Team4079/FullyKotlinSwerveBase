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
  CANCoderDriveStraightSteerSetPoint: Double,
) {
  private val driveMotor: TalonFX
  private val canCoder: CANcoder
  private val steerMotor: TalonFX

  private val positionSetter: PositionVoltage
  private val velocitySetter: VelocityTorqueCurrentFOC

  private val swerveModulePosition: SwerveModulePosition
  private var state: SwerveModuleState

  private var driveVelocity: Double
  private var drivePosition: Double
  private var steerPosition: Double
  private var steerVelocity: Double

  /**
   * Constructs a new SwerveModule.
   *
   * @param driveId The CAN ID of the drive motor.
   * @param steerId The CAN ID of the steer motor.
   * @param canCoderID The CAN ID of the CANcoder.
   * @param CANCoderDriveStraightSteerSetPoint The setpoint for the CANcoder when driving straight.
   */
  init {
    driveMotor = TalonFX(driveId)
    steerMotor = TalonFX(steerId)
    canCoder = CANcoder(canCoderID)

    positionSetter = PositionVoltage(0.0, 0.0, true, 0.0, 0, true, false, false).withSlot(0)
    velocitySetter = VelocityTorqueCurrentFOC(0.0)

    swerveModulePosition = SwerveModulePosition()
    state = SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0))

    val driveConfigs = TalonFXConfiguration()
    val steerConfigs = TalonFXConfiguration()
    val canCoderConfiguration = CANcoderConfiguration()

    driveConfigs.Slot0.kP = BasePIDGlobal.DRIVE_PID.p
    driveConfigs.Slot0.kI = BasePIDGlobal.DRIVE_PID.i
    driveConfigs.Slot0.kD = BasePIDGlobal.DRIVE_PID.d

    steerConfigs.Slot0.kP = BasePIDGlobal.STEER_PID.p
    steerConfigs.Slot0.kI = BasePIDGlobal.STEER_PID.i
    steerConfigs.Slot0.kD = BasePIDGlobal.STEER_PID.d

    driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
    steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake

    driveConfigs.MotorOutput.Inverted = SwerveGlobalValues.DRIVE_MOTOR_INVERETED
    steerConfigs.MotorOutput.Inverted = SwerveGlobalValues.STEER_MOTOR_INVERTED

    steerConfigs.Feedback.FeedbackRemoteSensorID = canCoderID
    steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
    steerConfigs.Feedback.RotorToSensorRatio = MotorGlobalValues.STEER_MOTOR_GEAR_RATIO
    steerConfigs.ClosedLoopGeneral.ContinuousWrap = true

    canCoderConfiguration.MagnetSensor.AbsoluteSensorRange =
      AbsoluteSensorRangeValue.Signed_PlusMinusHalf
    canCoderConfiguration.MagnetSensor.SensorDirection =
      SensorDirectionValue.CounterClockwise_Positive
    canCoderConfiguration.MagnetSensor.MagnetOffset =
      SwerveGlobalValues.ENCODER_OFFSET + CANCoderDriveStraightSteerSetPoint

    driveMotor.configurator.apply(driveConfigs)
    steerMotor.configurator.apply(steerConfigs)
    canCoder.configurator.apply(canCoderConfiguration)

    driveVelocity = driveMotor.velocity.valueAsDouble
    drivePosition = driveMotor.position.valueAsDouble
    steerVelocity = steerMotor.velocity.valueAsDouble
    steerPosition = steerMotor.position.valueAsDouble
  }

  /**
   * Gets the current position of the swerve module.
   *
   * @return The current position of the swerve module.
   */
  fun getPosition(): SwerveModulePosition {
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

  /**
   * Sets the desired state of the swerve module.
   *
   * @param state The desired state of the swerve module.
   * @param motor The motor associated with the swerve module.
   */
  fun setState(state: SwerveModuleState) { // SwerveSubsystem.Motor motor
    val newPosition = getPosition()
    // SmartDashboard.putNumber("desired state before optimize " + motor.name(),
    // state.angle.getDegrees());
    // SmartDashboard.putNumber("voltage " + motor.name(),
    // steerMotor.getMotorVoltage().getValueAsDouble());
    // SmartDashboard.putNumber("Applied " + motor.name(),
    // steerMotor.getSupplyCurrent().getValueAsDouble());
    val optimized = SwerveModuleState.optimize(state, newPosition.angle)

    val angleToSet = optimized.angle.rotations
    SmartDashboard.putNumber(
      "desired state after optimize " + canCoder.deviceID,
      optimized.angle.rotations,
    )

    // SmartDashboard.putNumber("current angle " + motor.name(), steerPosition);
    // SmartDashboard.putNumber("steer angle " + motor.name(),
    // steerMotor.getPosition().getValueAsDouble());
    steerMotor.setControl(positionSetter.withPosition(angleToSet))

    val velocityToSet =
      (optimized.speedMetersPerSecond *
        (MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO / MotorGlobalValues.METERS_PER_REVOLUTION))
    driveMotor.setControl(velocitySetter.withVelocity(velocityToSet))

    this.state = state
  }

  /**
   * Gets the current state of the swerve module.
   *
   * @return The current state of the swerve module.
   */
  fun getState(): SwerveModuleState {
    state.angle = Rotation2d.fromRotations(steerMotor.position.valueAsDouble)
    state.speedMetersPerSecond =
      (driveMotor.velocity.valueAsDouble *
        (MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO / MotorGlobalValues.METERS_PER_REVOLUTION))
    return state
  }
}
