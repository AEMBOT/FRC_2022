// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LimeLightTargeting;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(ShooterConstants.ShooterLeftMotor, ShooterConstants.ShooterRightMotor);

  private final LimeLightTargeting m_targeting = new LimeLightTargeting();

  private final XboxController m_Controller = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_Controller, XboxController.Button.kRightBumper.value).whileHeld(
      new RunCommand(() -> m_shooterSubsystem.incrementTargetPower(.05), m_shooterSubsystem));

      new JoystickButton(m_Controller, XboxController.Button.kLeftBumper.value).whileHeld(
        new RunCommand(() -> m_shooterSubsystem.incrementTargetPower(-.05), m_shooterSubsystem)
      );
      
      // NOTE: Doesn't have requirement of m_targeting subsystem. Could not figure out how to include
      // it. Can't add it as an additional argument for some reason, even though the function uses "..."
      // (variable-length arguments)
      new JoystickButton(m_Controller, XboxController.Button.kA.value).whileHeld(
        new RunCommand(() -> m_shooterSubsystem.shootFlywheels(m_targeting.getDistance()), m_shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand(
      () -> m_shooterSubsystem.runShooter(1));
  }
}
