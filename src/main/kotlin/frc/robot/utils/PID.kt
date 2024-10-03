@file:Suppress("unused")

package frc.robot.utils

/** A class representing a PID controller. */
class PID {
  var p: Double
  var i: Double
  var d: Double
  var f: Double = 0.0
  var s: Int = 0

  /**
   * Constructs a PID controller with the specified P, I, and D values.
   *
   * @param p The proportional gain.
   * @param i The integral gain.
   * @param d The derivative gain.
   */
  constructor(p: Double, i: Double, d: Double) {
    this.p = p
    this.i = i
    this.d = d
  }

  /**
   * Constructs a PID controller with the specified P, I, D, F values, and S value.
   *
   * @param p The proportional gain.
   * @param i The integral gain.
   * @param d The derivative gain.
   * @param f The feedforward gain.
   * @param s The S value.
   */
  constructor(p: Double, i: Double, d: Double, f: Double, s: Int) {
    this.p = p
    this.i = i
    this.d = d
    this.f = f
    this.s = s
  }

  /**
   * Constructs a PID controller with the specified P, I, D, and F values.
   *
   * @param p The proportional gain.
   * @param i The integral gain.
   * @param d The derivative gain.
   * @param f The feedforward gain.
   */
  constructor(p: Double, i: Double, d: Double, f: Double) {
    this.p = p
    this.i = i
    this.d = d
    this.f = f
    this.s = -1
  }

  /**
   * Updates the PID controller with the specified P, I, D, F, and S values.
   *
   * @param p The proportional gain.
   * @param i The integral gain.
   * @param d The derivative gain.
   * @param f The feedforward gain.
   * @param s The S value.
   */
  fun updatePID(p: Double, i: Double, d: Double, f: Double, s: Int) {
    this.p = p
    this.i = i
    this.d = d
    this.f = f
    this.s = s
  }

  /**
   * Updates the PID controller with the specified P, I, D, and F values.
   *
   * @param p The proportional gain.
   * @param i The integral gain.
   * @param d The derivative gain.
   * @param f The feedforward gain.
   */
  fun updatePID(p: Double, i: Double, d: Double, f: Double) {
    this.p = p
    this.i = i
    this.d = d
    this.f = f
    this.s = -1
  }

  /**
   * Updates the PID controller with the specified P, I, and D values.
   *
   * @param p The proportional gain.
   * @param i The integral gain.
   * @param d The derivative gain.
   */
  fun updatePID(p: Double, i: Double, d: Double) {
    this.p = p
    this.i = i
    this.d = d
    this.f = 0.0
    this.s = -1
  }

  var previousError: Double = 0.0
  var integral: Double = 0.0
  var setpoint: Double = 0.0
  var error: Double = 0.0
  private var output = 0.0

  /**
   * Calculates the output of the PID controller.
   *
   * @param actual The actual value.
   * @param setpoint The desired setpoint.
   * @return The calculated output.
   */
  fun calculate(actual: Double, setpoint: Double): Double {
    error = setpoint - actual // Error = Target - Actual
    this.integral +=
      (error * .02) // Integral is increased by the error*time (which is .02 seconds using normal
    val derivative = (error - this.previousError) / .02
    this.output = p * error + i * this.integral + d * derivative
    this.previousError = error
    return output
  }

  /**
   * Gets the current output of the PID controller.
   *
   * @return The current output.
   */
  fun getOutput(): Double {
    return output
  }

  /** Resets the integral term of the PID controller. */
  fun resetI() {
    this.integral = 0.0
  }
}
