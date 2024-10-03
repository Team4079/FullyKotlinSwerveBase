package frc.robot.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.GlobalsValues.MotorGlobalValues
import frc.robot.utils.GlobalsValues.SwerveGlobalValues
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal.pathFollower
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.kinematics
import java.util.Arrays
import java.util.function.BooleanSupplier
import java.util.function.Consumer
import java.util.function.Supplier

/** The [SwerveSubsystem] class includes all the motors to drive the robot. */
class SwerveSubsystem(photonvision: PhotonVision?) : SubsystemBase() {
  /** Pose estimator for the swerve drive. */
  private val poseEstimator: SwerveDrivePoseEstimator? = null // TODO: Set the pose estimator

  /** Field representation for the robot. */
  private val field: Field2d?

  /** Pigeon2 IMU for orientation. */
  private val pidgey: Pigeon2

  /** Array of swerve module states. */
  private val states: Array<SwerveModuleState?>

  /** Array of swerve modules. */
  private val modules: Array<SwerveModule>

  /** Photonvision instance for vision processing. */
  private val photonvision: PhotonVision?

  /** Rotation value for the robot. */
  private var rot = 0.0

  /** Flag to determine if the robot should invert its controls. */
  private val shouldInvert = false

  /** Creates a new DriveTrain. */
  init {
    modules =
      arrayOf<SwerveModule>(
        SwerveModule(
          MotorGlobalValues.FRONT_LEFT_DRIVE_ID,
          MotorGlobalValues.FRONT_LEFT_STEER_ID,
          MotorGlobalValues.FRONT_LEFT_CAN_CODER_ID,
          SwerveGlobalValues.CANCODER_VALUE9,
        ),
        SwerveModule(
          MotorGlobalValues.FRONT_RIGHT_DRIVE_ID,
          MotorGlobalValues.FRONT_RIGHT_STEER_ID,
          MotorGlobalValues.FRONT_RIGHT_CAN_CODER_ID,
          SwerveGlobalValues.CANCODER_VALUE10,
        ),
        SwerveModule(
          MotorGlobalValues.BACK_LEFT_DRIVE_ID,
          MotorGlobalValues.BACK_LEFT_STEER_ID,
          MotorGlobalValues.BACK_LEFT_CAN_CODER_ID,
          SwerveGlobalValues.CANCODER_VALUE11,
        ),
        SwerveModule(
          MotorGlobalValues.BACK_RIGHT_DRIVE_ID,
          MotorGlobalValues.BACK_RIGHT_STEER_ID,
          MotorGlobalValues.BACK_RIGHT_CAN_CODER_ID,
          SwerveGlobalValues.CANCODER_VALUE12,
        ),
      )

    field = Field2d()

    pidgey = Pigeon2(MotorGlobalValues.PIDGEY_ID)
    pidgey.reset()
    states = arrayOfNulls<SwerveModuleState>(4)
    this.photonvision = photonvision

    AutoBuilder.configureHolonomic(
      Supplier { this.getPose() }, // Robot pose supplier
      Consumer { pose: Pose2d?
        -> // Method to reset odometry (will be called if your auto has a starting pose)
        this.newPose(pose)
      },
      Supplier { this.getAutoSpeeds() }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      Consumer { chassisSpeeds: ChassisSpeeds?
        -> // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        this.chassisSpeedsDrive(chassisSpeeds!!)
      },
      pathFollower,
      BooleanSupplier {
        val alliance = DriverStation.getAlliance()
        if (alliance.isPresent) {
          if (shouldInvert) {
            alliance.get() == Alliance.Red
          } else {
            alliance.get() != Alliance.Blue
          }
        }
        false
      },
      this, // Reference to this subsystem
    )
  }

  /**
   * Sets the desired module states.
   *
   * @param states SwerveModuleState[]
   * @return void
   */
  fun setModuleStates(states: Array<SwerveModuleState?>) {
    for (i in states.indices) {
      modules[i].setState(states[i]!!)
    }
  }

  /**
   * Gets the module states.
   *
   * @param states SwerveModuleState[]
   * @return SwerveModuleState[]
   */
  fun getModuleStates(states: Array<SwerveModuleState?>): Array<SwerveModuleState?> {
    for (i in states.indices) {
      states[i] = modules[i].getState()
    }
    return states
  }

  /**
   * Gets the module positions.
   *
   * @param forwardSpeed double
   * @param leftSpeed double
   * @param turnSpeed double
   * @param isFieldOriented boolean
   * @return SwerveModulePosition[]
   */
  fun getDriveSpeeds(
    forwardSpeed: Double,
    leftSpeed: Double,
    turnSpeed: Double,
    isFieldOriented: Boolean,
  ) {
    var turnSpeed = turnSpeed

    SmartDashboard.putNumber("Forward speed", forwardSpeed)
    SmartDashboard.putNumber("Left speed", leftSpeed)

    turnSpeed = turnSpeed * MotorGlobalValues.TURN_CONSTANT

    var speeds =
      if (isFieldOriented) {
        ChassisSpeeds.fromFieldRelativeSpeeds(
          forwardSpeed,
          leftSpeed,
          turnSpeed,
          getPidgeyRotation(),
        )
      } else {
        ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed)
      }

    val states = kinematics.toSwerveModuleStates(speeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MotorGlobalValues.MAX_SPEED)
    setModuleStates(states)
  }

  /**
   * Gets the pidgey rotation.
   *
   * @return Rotation2d
   */
  fun getPidgeyRotation(): Rotation2d? {
    return pidgey.rotation2d
  }

  /**
   * Gets the pidgey angle.
   *
   * @return double
   */
  fun getHeading(): Double {
    return pidgey.angle
  }

  /**
   * Gets the pose.
   *
   * @return Pose2d
   */
  fun getPose(): Pose2d? {
    return poseEstimator!!.estimatedPosition
  }

  /**
   * Zeros the pose.
   *
   * @return void
   */
  fun zeroPose() {
    poseEstimator!!.resetPosition(
      Rotation2d.fromDegrees(getHeading()),
      getModulePositions(),
      Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
    )
  }

  /**
   * Resets the pose.
   *
   * @param pose Pose2d
   * @return void
   */
  fun newPose(pose: Pose2d?) {
    poseEstimator!!.resetPosition(pidgey.rotation2d, getModulePositions(), pose)
  }

  /**
   * Sets the field.
   *
   * @return void
   */
  fun getAutoSpeeds(): ChassisSpeeds? {
    val speeds = kinematics.toChassisSpeeds(*getModuleStates())
    return speeds
  }

  // TODO: Look at this code later
  /**
   * Gets the rotation pidggy.
   *
   * @return Rotation2d
   */
  fun getRotationPidggy(): Rotation2d {
    rot = -pidgey.rotation2d.degrees
    return Rotation2d.fromDegrees(rot)
  }

  /**
   * Drives the robot using the chassis speeds.
   *
   * @param chassisSpeeds ChassisSpeeds
   * @return void
   */
  fun chassisSpeedsDrive(chassisSpeeds: ChassisSpeeds) {
    val speeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getRotationPidggy())
    val states = kinematics.toSwerveModuleStates(speeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MotorGlobalValues.MAX_SPEED)
    setModuleStates(states)
  }

  /**
   * Gets the field.
   *
   * @return Field2d
   */
  private fun getModuleStates(): Array<SwerveModuleState?> {
    val moduleStates = arrayOfNulls<SwerveModuleState>(modules.size)
    for (i in modules.indices) {
      moduleStates[i] = modules[i].getState()
    }
    return moduleStates
  }

  /** Prints the module states */
  private fun printModuleStates() {
    Arrays.stream<SwerveModule?>(modules).forEach { module: SwerveModule? ->
      println(module!!.getState())
    }
  }

  /**
   * Gets the module positions.
   *
   * @return SwerveModulePosition[]
   */
  fun getModulePositions(): Array<SwerveModulePosition?> {
    val positions = arrayOfNulls<SwerveModulePosition>(states.size)

    for (i in modules.indices) {
      positions[i] = modules[i].getPosition()
    }

    return positions
  }
}
