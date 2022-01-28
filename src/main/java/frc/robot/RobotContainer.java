// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private TurnToAngleProfiled m_autoCommand;// = new TurnToAngleProfiled(10, m_robotDrive);

  // TODO: Move port to constants?
  private final XboxController m_driverController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default drivetrain command to arcade driving (happens during teleop)
    m_robotDrive.setDefaultCommand(
      new DefaultDrive(m_robotDrive, m_driverController::getLeftY, m_driverController::getRightX)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Turn to 10 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    // UNTESTED
    new JoystickButton(m_driverController, Button.kX.value)
    .whenPressed(new TurnToAngleProfiled(90, m_robotDrive).withTimeout(5));

    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(new TurnToAngleProfiled(-90, m_robotDrive).withTimeout(5));

    new JoystickButton(m_driverController, Button.kB.value)
      .whenPressed(new InstantCommand(() -> m_robotDrive.resetHeading()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Not sure whether we want to leave this here or not. Test with the joystick first
    //m_autoCommand = new TurnToAngleProfiled(10, m_robotDrive);
    return m_autoCommand;
  }
}
