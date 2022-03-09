// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbManual;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.TeleOpShooter;
import frc.robot.commands.autonomous.FiveBallAuto;
import frc.robot.commands.autonomous.TwoBallAuto;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.HomeOnHub;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
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
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final LimeLightTargeting m_limelight = new LimeLightTargeting();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  //Automodes - if you add more here, add them to the chooser in the container
  private TwoBallAuto m_autoCommand1 = new TwoBallAuto(m_robotDrive, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem, m_limelight);
  private FiveBallAuto m_autoCommand2 = new FiveBallAuto(m_robotDrive, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem, m_limelight);

  //Sets up driver controlled auto choices
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // TODO: Move port to constants?
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_secondaryController = new XboxController(1);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Set up chooser
    m_chooser.setDefaultOption("Two Ball Auto", m_autoCommand1);
    m_chooser.addOption("Five Ball Auto*", m_autoCommand2);
    SmartDashboard.putData(m_chooser);

    // Set default drivetrain command to arcade driving (happens during teleop)
    m_robotDrive.setDefaultCommand(
        new DefaultDrive(
            m_robotDrive, m_driverController::getLeftY, m_driverController::getRightX));

    m_climberSubsystem.setDefaultCommand(
        new ClimbManual(m_climberSubsystem, m_secondaryController::getYButtonPressed));            

    // Tried to write this without creating a separate file, but failed.
    // Please correct as some point

    // m_shooterSubsystem.setDefaultCommand(new InstantCommand(()-> m_shooterSubsystem.test(12.2),
    // m_shooterSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // PRIMARY CONTROLLER
    //Homing to Hub - A Button
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new HomeOnHub(m_limelight, m_robotDrive));

    //SECONDARY CONTROLLER
    // Shooter control based on limelight distance
    new JoystickButton(m_secondaryController, Button.kX.value)
        .whileHeld(new TeleOpShooter(m_shooterSubsystem));

    // toggle the exit side of the indexer
    new JoystickButton(m_secondaryController, Button.kB.value)
        .whenPressed(
            new InstantCommand(() -> m_indexerSubsystem.toggleExitSide(), m_indexerSubsystem));

    // Toggle the intake roller
    new JoystickButton(m_secondaryController, Button.kA.value)
        .whenPressed(new InstantCommand(() -> m_intakeSubsystem.toggleRoller(), m_intakeSubsystem));

    // Move the intake lift up
    new JoystickButton(m_secondaryController, Button.kLeftBumper.value)
        .whileHeld(new IntakeControl(m_intakeSubsystem, false));

    // Move the intake lift down
    new JoystickButton(m_secondaryController, Button.kRightBumper.value)
        .whileHeld(new IntakeControl(m_intakeSubsystem, true));
    
        // NOTE: Doesn't have requirement of m_targeting subsystem. Could not figure out how to include
    // it. Can't add it as an additional argument for some reason, even though the function uses
    // "..."
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
    return m_chooser.getSelected();
  }
}
