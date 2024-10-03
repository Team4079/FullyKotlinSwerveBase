// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.utils.GlobalsValues.SwerveGlobalValues

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
  private val swerveSubsystem: frc.robot.subsystems.SwerveSubsystem
  private val photonvision: frc.robot.subsystems.PhotonVision

  private val padA: JoystickButton
  private val padB: JoystickButton
  private val padX: JoystickButton
  private val padY: JoystickButton

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  init {
    frc.robot.utils.LogitechGamingPad(0).apply {
      padA = JoystickButton(this, 1)
      padB = JoystickButton(this, 2)
      padX = JoystickButton(this, 3)
      padY = JoystickButton(this, 4)

      photonvision = frc.robot.subsystems.PhotonVision()
      swerveSubsystem = frc.robot.subsystems.SwerveSubsystem(photonvision)
      swerveSubsystem.defaultCommand =
        frc.robot.commands.PadDrive(swerveSubsystem, this, SwerveGlobalValues.IS_FIELD_ORIENTATED)
    }

    configureBindings()
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * [edu.wpi.first.wpilibj2.command.button.Trigger] class, or our [JoystickButton] constructor with
   * an arbitrary predicate, or via the named factories in
   * [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
   * [edu.wpi.first.wpilibj2.command.button.CommandXboxController]/
   * [edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or
   * [edu.wpi.first.wpilibj2.command.button.CommandJoystick] Flight joysticks.
   */
  private fun configureBindings() {
    // padA.onTrue(new InstantCommand(swerveSubsystem::addRotorPositionsforModules));
    // padB.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    // padY.onTrue(new InstantCommand(swerveSubsystem::configAAcornMode));
    // padX.onTrue(new InstantCommand(swerveSubsystem::configSlowMode));
  }

  /**
   * Use this to pass the autonomous command to the main [Robot] class.
   *
   * @return the command to run in autonomous
   */
  fun getAutonomousCommand(): Command {
    return com.pathplanner.lib.commands.PathPlannerAuto("Straight Auto")
  }
}
