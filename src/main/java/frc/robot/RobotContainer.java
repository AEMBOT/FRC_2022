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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbManual;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.TeleOpShooter;
import frc.robot.commands.autonomous.FiveBallAuto;
import frc.robot.commands.autonomous.TwoBallAuto;
import frc.robot.commands.drive.AlignWithHub;
import frc.robot.commands.drive.AlignWithHubSmart;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.HomeOnHub;
import frc.robot.commands.drive.TurnToAngleProfiled;
import frc.robot.commands.drive.TurnToAngleSmart;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.RampThenShoot;
import frc.robot.commands.utilities.TimedRumble;
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
        new ClimbManual(m_climberSubsystem, m_driverController::getStartButtonPressed));

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
        .whenPressed(new AlignWithHubSmart(m_limelight, m_robotDrive));
        // .whenPressed(new AlignWithHub(m_robotDrive, m_limelight).withTimeout(0.5));

    //SECONDARY CONTROLLER
    // Shooter control based on limelight distance
    // new JoystickButton(m_secondaryController, Button.kX.value)
    //     .whileHeld(new TeleOpShooter(m_shooterSubsystem));

    // Ramps up the shooter then runs the upper indexer into it
    // TODO: the limelight needs to be turned on before checking if it has a valid target
    new JoystickButton(m_secondaryController, Button.kB.value)
        .whileHeld(
          new ConditionalCommand(new RampThenShoot(m_indexerSubsystem, m_shooterSubsystem),
            new ParallelCommandGroup(
              new RampThenShoot(m_indexerSubsystem, m_shooterSubsystem),
              new TimedRumble(m_driverController, 0.5, 0.5)),
            m_limelight::hasValidTarget));

    // Run the intake roller when A is held
    new JoystickButton(m_secondaryController, Button.kA.value)
        .whileHeld(new RunIntake(m_intakeSubsystem));

    // Move the intake lift up
    new JoystickButton(m_secondaryController, Button.kLeftBumper.value)
        .whileHeld(new IntakeControl(m_intakeSubsystem, false));

    // Move the intake lift down
    new JoystickButton(m_secondaryController, Button.kRightBumper.value)
        .whileHeld(new IntakeControl(m_intakeSubsystem, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Returns the autonomous command selected on the dashboard
    return m_chooser.getSelected();
  }
}
