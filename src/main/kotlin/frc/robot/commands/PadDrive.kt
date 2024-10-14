package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.utils.GlobalsValues
import frc.robot.utils.LogitechGamingPad
import frc.robot.utils.dashNumbers

/**
 * A command to drive the robot using a Logitech gaming pad.
 *
 * @param swerveSubsystem The swerve subsystem used to control the robot's movement.
 * @param pad The Logitech gaming pad used to control the robot.
 * @param isFieldOriented Whether the driving is field-oriented.
 */
class PadDrive(
  private val swerveSubsystem: SwerveSubsystem,
  private val pad: LogitechGamingPad,
  private val isFieldOriented: Boolean,
) : Command() {

  /** Initializes the command and adds the swerve subsystem as a requirement. */
  override fun initialize() {
    addRequirements(this.swerveSubsystem)
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. Reads joystick values and
   * sends them to the swerve subsystem to control the robot.
   */
  override fun execute() {
    val y = -pad.getLeftAnalogYAxis() * GlobalsValues.MotorGlobalValues.MAX_SPEED
    val x = -pad.getLeftAnalogXAxis() * GlobalsValues.MotorGlobalValues.MAX_SPEED
    val rotation = pad.getRightAnalogXAxis() * GlobalsValues.MotorGlobalValues.MAX_ANGULAR_SPEED

    dashNumbers("Y Joystick" to y, "X Joystick" to x, "Rotation Joystick" to rotation)

    swerveSubsystem.getDriveSpeeds(y, x, rotation, isFieldOriented)
  }

  /**
   * Returns true when the command should end.
   *
   * @return Always returns false, as this command never ends on its own.
   */
  override fun isFinished(): Boolean {
    return false
  }
}
