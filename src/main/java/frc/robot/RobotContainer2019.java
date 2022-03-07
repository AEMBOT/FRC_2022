// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveStraightProfiled;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer2019 extends RobotContainer {
  // Start with a constants instance
  private final Constants constants = new Constants();

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(constants.new DriveConstants());

  private DriveStraightProfiled m_autoCommand = new DriveStraightProfiled(-1.0, m_robotDrive, constants.new DriveConstants().new StraightPID());

  // TODO: Move port to constants?
  private final XboxController m_driverController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer2019() {
    // Configure the button binding
    configureButtonBindings();

    // Set default drivetrain command to arcade driving (happens during teleop)
    m_robotDrive.setDefaultCommand(
        new DefaultDrive(
            m_robotDrive, m_driverController::getLeftY, m_driverController::getRightX));

    // Tried to write this without creating a separate file, but failed.
    // Please correct as some point

    //m_shooterSubsystem.setDefaultCommand(new InstantCommand(()-> m_shooterSubsystem.test(12.2), m_shooterSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Turn left 90 degrees with a 3 second timeout
      
    // NOTE: Doesn't have requirement of m_targeting subsystem. Could not figure out how to include
    // it. Can't add it as an additional argument for some reason, even though the function uses "..."
    // (variable-length arguments)
    /*
    RunCommand targetCommand = new RunCommand(() -> m_shooterSubsystem.shootFlywheels(m_targeting.getDistance()));
    targetCommand.addRequirements(m_targeting,m_shooterSubsystem);

    new JoystickButton(m_Controller, XboxController.Button.kA.value).whileHeld(targetCommand);

    */


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Not sure whether we want to leave this here or not. Test with the joystick first
    // m_autoCommand = new TurnToAngleProfiled(10, m_robotDrive);
    return m_autoCommand;
  }
}
