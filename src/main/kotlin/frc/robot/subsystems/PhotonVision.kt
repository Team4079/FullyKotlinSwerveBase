package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.GlobalsValues
import java.util.Optional
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.PhotonUtils
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

/** The PhotonVision subsystem handles vision processing using PhotonVision cameras. */
@Suppress("unused")
class PhotonVision : SubsystemBase() {
  // PhotonVision cameras
  var camera1: PhotonCamera = PhotonCamera("Camera One")
  var camera2: PhotonCamera = PhotonCamera("Camera Two")

  // Tracked targets from the cameras
  var target1: PhotonTrackedTarget? = null // TODO: Use this and the below to track targets
  var target2: PhotonTrackedTarget? = null

  // Pose estimator for determining the robot's position on the field
  var photonPoseEstimator: PhotonPoseEstimator

  // AprilTag field layout for the 2024 Crescendo field
  var aprilTagFieldLayout: AprilTagFieldLayout? =
    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

  // Transformation from the robot to the camera
  var robotToCam: Transform3d =
    Transform3d(
      Translation3d(0.5, 0.0, 0.5),
      Rotation3d(0.0, 0.0, 0.0),
    ) // Cam mounted facing forward, half a meter forward of center, half a meter up from
  // center.

  // Variables to store target visibility and pose information for camera 1
  var targetVisible1: Boolean = false
  var targetYaw1: Double = 0.0
  var targetPoseAmbiguity1: Double = 0.0
  var range1: Double = 0.0

  // Variables to store target visibility and pose information for camera 2
  var targetVisible2: Boolean = false
  var targetYaw2: Double = 0.0
  var targetPoseAmbiguity2: Double = 0.0
  var range2: Double = 0.0

  // Variables to store the selected target's yaw and range
  var targetYaw: Double = 0.0
  var rangeToTarget: Double = 0.0

  /** Constructs a new PhotonVision subsystem. */
  init {
    photonPoseEstimator =
      PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera1,
        robotToCam,
      )
  }

  /**
   * This method is called periodically by the scheduler. It updates the tracked targets and
   * displays relevant information on the SmartDashboard.
   */
  override fun periodic() {
    val result1 = camera1.getLatestResult()
    val result2 = camera2.getLatestResult()

    processCameraResult(result1, 1)
    processCameraResult(result2, 2)

    if (targetPoseAmbiguity1 > targetPoseAmbiguity2) {
      targetYaw = targetYaw1
      rangeToTarget = range1
    } else {
      targetYaw = targetYaw2
      rangeToTarget = range2
    }
  }

  /**
   * Processes the result from a camera and updates the target information.
   *
   * @param result The result from the camera's pipeline.
   * @param cameraIndex The index of the camera (1 or 2).
   */
  private fun processCameraResult(result: PhotonPipelineResult, cameraIndex: Int) {
    if (result.hasTargets()) {
      for (tag in result.getTargets()) {
        if (tag.fiducialId == 7) {
          if (cameraIndex == 1) {
            targetPoseAmbiguity1 = tag.poseAmbiguity
            targetYaw1 = tag.yaw
            targetVisible1 = true
            range1 =
              calculateRange(
                tag,
                GlobalsValues.PhotonVisionConstants.CAMERA_ONE_HEIGHT,
                GlobalsValues.PhotonVisionConstants.CAMERA_ONE_ANGLE,
              )
          } else {
            targetPoseAmbiguity2 = tag.poseAmbiguity
            targetYaw2 = tag.yaw
            targetVisible2 = true
            range2 =
              calculateRange(
                tag,
                GlobalsValues.PhotonVisionConstants.CAMERA_TWO_HEIGHT,
                GlobalsValues.PhotonVisionConstants.CAMERA_TWO_ANGLE,
              )
          }
        }
      }
    } else {
      if (cameraIndex == 1) {
        targetVisible1 = false
        targetPoseAmbiguity1 = 1e9
      } else {
        targetVisible2 = false
        targetPoseAmbiguity2 = 1e9
      }
    }
  }

  /**
   * Calculates the range to a target based on the camera height, target height, and camera angle.
   *
   * @param tag The tracked target.
   * @param cameraHeight The height of the camera.
   * @param cameraAngle The angle of the camera.
   * @return The calculated range to the target.
   */
  private fun calculateRange(
    tag: PhotonTrackedTarget,
    cameraHeight: Double,
    cameraAngle: Double,
  ): Double {
    return PhotonUtils.calculateDistanceToTargetMeters(
      cameraHeight,
      1.435, // From 2024 game manual for ID 7 | IMPORTANT TO CHANGE
      cameraAngle, // Rotation about Y = Pitch | UP IS POSITIVE
      Units.degreesToRadians(tag.pitch),
    )
  }

  /**
   * Gets the estimated global pose of the robot.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot.
   * @return An Optional containing the estimated robot pose, or empty if no pose could be
   *   estimated.
   */
  fun getEstimatedGlobalPose(prevEstimatedRobotPose: Pose2d?): Optional<EstimatedRobotPose?>? {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose)
    return photonPoseEstimator.update()
  }
}
