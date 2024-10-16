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

/**
 * The SwerveSubsystem class represents the swerve drive subsystem of the robot.
 *
 * @param photonvision An optional PhotonVision instance for vision processing.
 */
@Suppress("MemberVisibilityCanBePrivate", "unused", "kotlin:S6619")
class SwerveSubsystem(photonvision: PhotonVision?) : SubsystemBase() {
  /** Pose estimator for the swerve drive */
  private val poseEstimator: SwerveDrivePoseEstimator? = null // TODO: Implement this

  /** Field representation for the SmartDashboard */
  private val field: Field2d = Field2d()

  /** Pigeon2 gyroscope for the swerve drive */
  private val pidgey: Pigeon2 = Pigeon2(MotorGlobalValues.PIDGEY_ID)

  /** Array of swerve module states */
  private val states: Array<SwerveModuleState?>

  /** Array of swerve modules */
  private val modules: Array<SwerveModule> =
    arrayOf(
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

  /** Optional PhotonVision instance */
  private val photonvision: PhotonVision?

  /** Rotation value */
  private var rot = 0.0

  /** Flag to determine if the drive should be inverted */
  private val shouldInvert = false

  /** Array of swerve module positions */
  val modulePositions: Array<SwerveModulePosition?>
    get() = modules.map { it.position }.toTypedArray()

  /** Array of swerve module states */
  var moduleStates: Array<SwerveModuleState?>
    get() = modules.map { it.state }.toTypedArray()
    set(states) = states.forEachIndexed { i, state -> modules[i].state = state!! }

  /** Rotation of the Pigeon2 gyroscope */
  val pidgeyRotation: Rotation2d
    get() = pidgey.rotation2d

  /**
   * Gets the heading of the robot.
   *
   * @return The heading in degrees.
   */
  fun getHeading(): Double {
    return pidgey.angle
  }

  /** Current pose of the robot */
  val pose: Pose2d?
    get() = poseEstimator!!.estimatedPosition

  /** Chassis speeds for autonomous driving */
  val autoSpeeds: ChassisSpeeds?
    get() {
      val speeds = kinematics.toChassisSpeeds(*moduleStates)
      return speeds
    }

  /** Rotation of the Pigeon2 gyroscope for PID control */
  val rotationPidggy: Rotation2d
    get() {
      rot = -pidgey.rotation2d.degrees
      return Rotation2d.fromDegrees(rot)
    }

  init {
    pidgey.reset()
    states = arrayOfNulls(moduleStates.size)
    this.photonvision = photonvision

    AutoBuilder.configureHolonomic(
      { this.pose },
      { pose: Pose2d? -> this.newPose(pose) },
      { this.autoSpeeds },
      { chassisSpeeds: ChassisSpeeds? -> this.chassisSpeedsDrive(chassisSpeeds!!) },
      pathFollower,
      {
        val alliance = DriverStation.getAlliance()
        if (alliance.isPresent) {
          (shouldInvert && alliance.get() == Alliance.Red) ||
            (!shouldInvert && alliance.get() != Alliance.Blue)
        } else {
          false
        }
      },
      this,
    )
  }

  /**
   * Sets the drive speeds for the swerve modules.
   *
   * @param forwardSpeed The forward speed.
   * @param leftSpeed The left speed.
   * @param turnSpeed The turn speed.
   * @param isFieldOriented Whether the drive is field-oriented.
   */
  fun getDriveSpeeds(
    forwardSpeed: Double,
    leftSpeed: Double,
    turnSpeed: Double,
    isFieldOriented: Boolean,
  ) {
    var turnSpeed1 = turnSpeed

    SmartDashboard.putNumber("Forward speed", forwardSpeed)
    SmartDashboard.putNumber("Left speed", leftSpeed)

    turnSpeed1 *= MotorGlobalValues.TURN_CONSTANT

    val speeds =
      if (isFieldOriented) {
        ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, leftSpeed, turnSpeed1, pidgeyRotation)
      } else {
        ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed1)
      }

    val states = kinematics.toSwerveModuleStates(speeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MotorGlobalValues.MAX_SPEED)
    moduleStates = states
  }

  /** Resets the pose of the robot to zero. */
  fun zeroPose() {
    poseEstimator!!.resetPosition(
      Rotation2d.fromDegrees(getHeading()),
      modulePositions,
      Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
    )
  }

  /**
   * Sets a new pose for the robot.
   *
   * @param pose The new pose.
   */
  fun newPose(pose: Pose2d?) {
    poseEstimator!!.resetPosition(pidgey.rotation2d, modulePositions, pose)
  }

  /**
   * Drives the robot using chassis speeds.
   *
   * @param chassisSpeeds The chassis speeds.
   */
  fun chassisSpeedsDrive(chassisSpeeds: ChassisSpeeds) {
    val speeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, rotationPidggy)
    val states = kinematics.toSwerveModuleStates(speeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MotorGlobalValues.MAX_SPEED)
    moduleStates = states
  }
}
