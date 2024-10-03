@file:Suppress("unused")

package frc.robot.utils

import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController

/**
 * Represents a Logitech Gaming Pad controller.
 *
 * @param usbPort The USB port number to which the controller is connected.
 */
class LogitechGamingPad(usbPort: Int) : XboxController(usbPort) {
  private val gamepad: Joystick = Joystick(usbPort)

  /** Enum representing the D-Pad directions and their corresponding angles. */
  enum class DPad(private val angle: Int) {
    UP(0),
    UP_RIGHT(45),
    RIGHT(90),
    DOWN_RIGHT(135),
    DOWN(180),
    DOWN_LEFT(225),
    LEFT(270),
    UP_LEFT(315);

    /**
     * Gets the angle associated with the D-Pad direction.
     *
     * @return The angle of the D-Pad direction.
     */
    fun getAngle(): Int {
      return angle
    }
  }

  /** Enum representing the buttons on the controller and their corresponding button numbers. */
  enum class Button(private val button: Int) {
    A(1),
    B(2),
    X(3),
    Y(4),
    LEFT_BUMPER(5),
    RIGHT_BUMPER(6),
    BACK(7),
    START(8);

    /**
     * Gets the button number associated with the button.
     *
     * @return The button number.
     */
    fun getButton(): Int {
      return button
    }
  }

  /** Enum representing the analog axes on the controller and their corresponding axis numbers. */
  enum class Axis(private val axis: Int) {
    LEFT_ANALOG_X(0),
    LEFT_ANALOG_Y(1),
    RIGHT_ANALOG_X(4),
    RIGHT_ANALOG_Y(5);

    /**
     * Gets the axis number associated with the analog axis.
     *
     * @return The axis number.
     */
    fun getAxis(): Int {
      return axis
    }
  }

  /** Enum representing the triggers on the controller and their corresponding trigger numbers. */
  enum class Trigger(private val trigger: Int) {
    LEFT(2),
    RIGHT(3);

    /**
     * Gets the trigger number associated with the trigger.
     *
     * @return The trigger number.
     */
    fun getTrigger(): Int {
      return trigger
    }
  }

  /**
   * Checks if the left bumper is pressed.
   *
   * @return True if the left bumper is pressed, false otherwise.
   */
  override fun getLeftBumper() = gamepad.getRawButton(Button.LEFT_BUMPER.getButton())

  /**
   * Gets the value of the left trigger.
   *
   * @return The value of the left trigger.
   */
  fun getLeftTriggerValue() = gamepad.getRawAxis(Trigger.LEFT.getTrigger())

  /**
   * Gets the X-axis value of the left analog stick.
   *
   * @return The X-axis value of the left analog stick.
   */
  fun getLeftAnalogXAxis() = gamepad.getRawAxis(Axis.LEFT_ANALOG_X.getAxis())

  /**
   * Gets the Y-axis value of the left analog stick.
   *
   * @return The Y-axis value of the left analog stick.
   */
  fun getLeftAnalogYAxis() = gamepad.getRawAxis(Axis.LEFT_ANALOG_Y.getAxis())

  /**
   * Checks if the right bumper is pressed.
   *
   * @return True if the right bumper is pressed, false otherwise.
   */
  override fun getRightBumper() = gamepad.getRawButton(Button.RIGHT_BUMPER.getButton())

  /**
   * Gets the value of the right trigger.
   *
   * @return The value of the right trigger.
   */
  fun getRightTriggerValue() = gamepad.getRawAxis(Trigger.RIGHT.getTrigger())

  /**
   * Gets the X-axis value of the right analog stick.
   *
   * @return The X-axis value of the right analog stick.
   */
  fun getRightAnalogXAxis() = gamepad.getRawAxis(Axis.RIGHT_ANALOG_X.getAxis())

  /**
   * Gets the Y-axis value of the right analog stick.
   *
   * @return The Y-axis value of the right analog stick.
   */
  fun getRightAnalogYAxis() = gamepad.getRawAxis(Axis.RIGHT_ANALOG_Y.getAxis())

  /**
   * Checks if the A button is pressed.
   *
   * @return True if the A button is pressed, false otherwise.
   */
  override fun getAButton() = gamepad.getRawButton(Button.A.getButton())

  /**
   * Checks if the B button is pressed.
   *
   * @return True if the B button is pressed, false otherwise.
   */
  override fun getBButton() = gamepad.getRawButton(Button.B.getButton())

  /**
   * Checks if the X button is pressed.
   *
   * @return True if the X button is pressed, false otherwise.
   */
  override fun getXButton() = gamepad.getRawButton(Button.X.getButton())

  /**
   * Checks if the Y button is pressed.
   *
   * @return True if the Y button is pressed, false otherwise.
   */
  override fun getYButton() = gamepad.getRawButton(Button.Y.getButton())

  /**
   * Checks if the Back button is pressed.
   *
   * @return True if the Back button is pressed, false otherwise.
   */
  override fun getBackButton() = gamepad.getRawButton(Button.BACK.getButton())

  /**
   * Checks if the Start button is pressed.
   *
   * @return True if the Start button is pressed, false otherwise.
   */
  override fun getStartButton() = gamepad.getRawButton(Button.START.getButton())

  /**
   * Checks if the D-Pad is pressed in a specific direction.
   *
   * @param index The index of the D-Pad direction (0-7).
   * @return True if the D-Pad is pressed in the specified direction, false otherwise.
   */
  fun checkDPad(index: Int) = (0 <= index && index <= 7) && (index * 45) == gamepad.pov

  /**
   * Checks if the D-Pad is pressed at a specific angle.
   *
   * @param angle The angle to check.
   * @param inDegrees True if the angle is in degrees, false if in radians.
   * @return True if the D-Pad is pressed at the specified angle, false otherwise.
   */
  fun checkDPad(angle: Double, inDegrees: Boolean): Boolean {
    var angle = angle
    if (!inDegrees) angle = Math.toDegrees(angle)
    return angle.toInt() == gamepad.pov
  }

  /**
   * Gets the current D-Pad direction.
   *
   * @return The current D-Pad direction, or -1 if not pressed.
   */
  fun getDPad() = if (gamepad.pov == -1) gamepad.pov else gamepad.pov / 45

  /**
   * Gets the current D-Pad direction.
   *
   * @param inDegrees True to get the direction in degrees, false to get in radians.
   * @return The current D-Pad direction in the specified unit.
   */
  fun getDPad(inDegrees: Boolean) =
    if (inDegrees) gamepad.pov.toDouble() else Math.toRadians(gamepad.pov.toDouble())

  /**
   * Checks if the D-Pad is pressed.
   *
   * @return True if the D-Pad is pressed, false otherwise.
   */
  fun dPadIsPressed() = gamepad.pov != -1

  /**
   * Sets the rumble amount for the controller.
   *
   * @param amount The rumble amount (0.0 to 1.0).
   */
  fun setRumble(amount: Float) {
    gamepad.setRumble(RumbleType.kLeftRumble, amount.toDouble())
    gamepad.setRumble(RumbleType.kRightRumble, amount.toDouble())
  }

  /**
   * Gets the raw axis value.
   *
   * @param which The axis number.
   * @return The raw axis value.
   */
  override fun getRawAxis(which: Int) = gamepad.getRawAxis(which)

  /**
   * Gets the raw button state.
   *
   * @param button The button number.
   * @return True if the button is pressed, false otherwise.
   */
  override fun getRawButton(button: Int) = gamepad.getRawButton(button)

  /**
   * Gets the POV value.
   *
   * @param pov The POV number.
   * @return The POV value.
   */
  override fun getPOV(pov: Int) = gamepad.getPOV(pov)

  /**
   * Gets the twist value of the joystick.
   *
   * @return The twist value.
   */
  fun getTwist(): Double {
    return 0.0
  }

  /**
   * Gets the throttle value of the joystick.
   *
   * @return The throttle value.
   */
  fun getThrottle(): Double {
    return 0.0
  }
}
