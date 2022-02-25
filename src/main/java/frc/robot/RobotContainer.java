// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveStraightProfiled;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TimeRotation;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightTargeting;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(ShooterConstants.LeftMotorCANId, ShooterConstants.RightMotorCANId);
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private TimeRotation m_timeRotation =
      new TimeRotation(0.3, m_robotDrive); // = new TurnToAngleProfiled(10, m_robotDrive);
  private DriveStraightProfiled m_autoCommand = new DriveStraightProfiled(-1.0, m_robotDrive);
  private final LimeLightTargeting m_targeting = new LimeLightTargeting();

  private final XboxController m_Controller = new XboxController(0);

  // TODO: Move port to constants?
  private final XboxController m_driverController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default drivetrain command to arcade driving (happens during teleop)
    m_robotDrive.setDefaultCommand(
        new DefaultDrive(
            m_robotDrive, m_driverController::getLeftY, m_driverController::getRightX));

    // Tried to write this without creating a separate file, but failed.
    // Please correct as some point
    m_shooterSubsystem.setDefaultCommand(new ShooterCommand(m_shooterSubsystem, m_driverController::getRightTriggerAxis));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Turn left 90 degrees with a 3 second timeout
    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new TurnToAngleProfiled(90, m_robotDrive).withTimeout(3));

    // Turn right 90 degrees with a 3 second timeout
    new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(new TurnToAngleProfiled(-90, m_robotDrive).withTimeout(3));

    // Reset the robot's heading & odometry
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(new InstantCommand(() -> m_robotDrive.resetHeading(), m_robotDrive));

    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new InstantCommand(() -> m_intakeSubsystem.toggleRoller(), m_intakeSubsystem));

    new JoystickButton(m_Controller, XboxController.Button.kLeftBumper.value).whileHeld(
      new RunCommand(() -> m_shooterSubsystem.incrementTargetPower(-.05), m_shooterSubsystem)
    );

    new JoystickButton(m_driverController, Button.kRightBumper.value)
    .whileHeld(new IntakeControl(m_intakeSubsystem, false));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
    .whileHeld(new IntakeControl(m_intakeSubsystem, true));
      
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
