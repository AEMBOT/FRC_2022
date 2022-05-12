// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.FollowTrajectory;
import frc.robot.commands.autonomous.TaxiThenShoot;
import frc.robot.commands.autonomous.TrajectoryCommandGroup;
import frc.robot.commands.autonomous.TrajectoryCommandGroup.PositionTrigger;
import frc.robot.commands.climber.ClimbTimed;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.IntakeCargo;
import frc.robot.commands.intake.LiftIntake;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RunIntakeRoller;
import frc.robot.commands.shooter.RampThenShoot;
import frc.robot.commands.utilities.Noop;
import frc.robot.commands.utilities.enums.CargoDirection;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.LEDMode;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Limelight doesn't warrant being a subsystem for how we're using it
  private final Limelight m_limelight = new Limelight();

  // Robot subsystems (see subsystems/ folder for more info)
  private final DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_limelight);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  // Controllers (we use Xbox ones)
  private final XboxController m_driverController = new XboxController(kDriverPort);
  private final XboxController m_secondaryController = new XboxController(kSecondaryPort);

  // Power Distribution Panel (PDP) and Pneumatics Control Module (PCM)
  private final PowerDistribution m_pdp = new PowerDistribution();
  private final PneumaticsControlModule m_pcm = new PneumaticsControlModule();

  // Used for toggling the compressor state (see updateCompressorState() method)
  private boolean m_compressorEnabled = true;

  // Automodes - if you add more here, add them to the chooser in setupAutoChooser()
  private final TaxiThenShoot m_taxiThenShoot =
      new TaxiThenShoot(
          m_robotDrive, m_intakeSubsystem, m_indexerSubsystem, m_shooterSubsystem, m_limelight);
  private final FollowTrajectory m_testTrajectory =
      new FollowTrajectory(
          m_robotDrive,
          PathPlanner.loadPath(
              "Test Path", kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecondSquared));
  private final Command m_intakeAlongTrajectory =
      new LiftIntake(m_intakeSubsystem)
          .andThen(new LowerIntake(m_intakeSubsystem))
          .andThen(
              new TrajectoryCommandGroup(
                  PathPlanner.loadPath(
                      "Squiggle",
                      kMaxVelocityMetersPerSecond,
                      kMaxAccelerationMetersPerSecondSquared),
                  m_robotDrive,
                  Map.of(
                      // Enable the intake roller for a bit towards the end of the path
                      new IntakeCargo(m_indexerSubsystem, m_intakeSubsystem).withTimeout(1.5),
                      new PositionTrigger(new Translation2d(3, 3.5), 0.5))));

  // Sets up driver controlled auto choices
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Setup the Limelight & USB camera
    setupCameras();

    // Display an autonomous chooser on the dashboard
    setupAutoChooser();

    // Disable LiveWindow telemetry, since we don't use it at all
    LiveWindow.disableAllTelemetry();

    // Silence joystick connection warnings since they're not useful
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set default drivetrain command to arcade driving (happens during teleop)
    m_robotDrive.setDefaultCommand(
        new DefaultDrive(
            m_robotDrive, m_driverController::getLeftY, m_driverController::getRightX));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // PRIMARY CONTROLLER
    // Homing to Hub - A Button
    // new JoystickButton(m_driverController, Button.kA.value)
    //     .whenPressed(new AlignWithHub(m_limelight, m_robotDrive).withTimeout(1.5));

    // Climb sequence - Start Button
    // new JoystickButton(m_driverController, Button.kStart.value)
    //     .whenPressed(new ClimbTimed(m_climberSubsystem, m_driverController::getStartButtonPressed));

    // SECONDARY CONTROLLER

    // Ramps up the shooter then runs the upper indexer into it - B
    new JoystickButton(m_secondaryController, Button.kB.value)
        .whileHeld(
            new RampThenShoot(
                m_intakeSubsystem,
                m_indexerSubsystem,
                m_shooterSubsystem,
                m_limelight,
                m_driverController,
                m_secondaryController));

    // Operate the intake lift - Left/Right bumper
    new JoystickButton(m_secondaryController, Button.kRightBumper.value)
        .whenPressed(new LiftIntake(m_intakeSubsystem));

    new JoystickButton(m_secondaryController, Button.kLeftBumper.value)
        .whileHeld(new LowerIntake(m_intakeSubsystem));

    // Moving the climber is only possible when both secondary triggers are pressed
    Trigger dpadUp = new Trigger(() -> m_secondaryController.getPOV() == 0);
    Trigger dpadDown = new Trigger(() -> m_secondaryController.getPOV() == 180);

    // Deploy/retract climber - up/down on dpad
    dpadUp.whenActive(m_climberSubsystem::extendArms, m_climberSubsystem);
    dpadDown.whenActive(m_climberSubsystem::retractArms, m_climberSubsystem);

    // dpadUp.whenActive(m_shooterSubsystem::incrementRPMOffset, m_shooterSubsystem);
    // dpadDown.whenActive(m_shooterSubsystem::decrementRPMOffset, m_shooterSubsystem);

    // Run the intake roller & lower indexer belts to intake cargo - A
    new JoystickButton(m_secondaryController, Button.kA.value)
        .whileHeld(m_indexerSubsystem::intakeCargo, m_indexerSubsystem)
        .whenReleased(m_indexerSubsystem::stopBelts, m_indexerSubsystem);

    // Eject any cargo in the indexer/intake - X
    new JoystickButton(m_secondaryController, Button.kX.value)
        .whileHeld(m_indexerSubsystem::moveCargoDown, m_indexerSubsystem)
        .whenReleased(m_indexerSubsystem::stopBelts, m_indexerSubsystem);
  }

  /** Configures the secondary USB camera & Limelight port forwarding. */
  private void setupCameras() {
    // Turn off the limelight's LEDs so we don't blind the children
    m_limelight.setLEDMode(LEDMode.ForceOff);

    // Set up USB (rear-facing) camera
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 240);

    // Forward limelight ports over USB
    PortForwarder.add(5800, "10.64.43.11", 5800);
    PortForwarder.add(5801, "10.64.43.11", 5801);
    PortForwarder.add(5802, "10.64.43.11", 5802);
    PortForwarder.add(5803, "10.64.43.11", 5803);
    PortForwarder.add(5804, "10.64.43.11", 5804);
    PortForwarder.add(5805, "10.64.43.11", 5805);
  }

  /** Sets up the autonomous chooser on the dashboard. */
  private void setupAutoChooser() {
    // IMPORTANT: Add any automodes here, don't override the chooser
    m_autoChooser.setDefaultOption("Taxi & Shoot", m_taxiThenShoot);
    m_autoChooser.addOption("Follow Trajectory", m_testTrajectory);
    m_autoChooser.addOption("Intake along trajectory", m_intakeAlongTrajectory);

    // Display the chooser on the dashboard (but not during a demo)
    // SmartDashboard.putData(m_autoChooser);
  }

  /** Used to toggle the compressor using the dashboard. */
  public void updateCompressorStatus() {
    m_compressorEnabled = SmartDashboard.getBoolean("Enable Compressor", m_compressorEnabled);
    if (m_compressorEnabled) {
      m_pcm.enableCompressorDigital();
    } else {
      m_pcm.disableCompressor();
    }

    // Put the toggle onto the dashboard
    SmartDashboard.putBoolean("Enable Compressor", m_compressorEnabled);
  }

  /** Retracts the climber pistons and sets them to a vertical position. */
  public void resetClimber() {
    m_climberSubsystem.retractArms();
    m_climberSubsystem.setPistonsVertical();
  }

  /** Clears all sticky faults on the PCM and PDP. */
  public void clearAllStickyFaults() {
    m_pdp.clearStickyFaults();
    m_pcm.clearAllStickyFaults();
  }

  /**
   * Resets all subsystems to a stable state, with motors not moving and PID setpoints to zero, if
   * applicable. Meant to be called in {@link Robot#disabledInit()}.
   */
  public void resetSubsystems() {
    // Stop drive motors & reset internal feedforward
    m_robotDrive.stopMotors();
    m_robotDrive.resetFeedforward();

    // Stop running the shooter flywheels
    m_shooterSubsystem.stopShooter();

    // Stop the indexer belts
    m_indexerSubsystem.stopBelts();

    // Stop the intake roller/lift
    m_intakeSubsystem.stopRoller();
    m_intakeSubsystem.stopLift();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Returns the autonomous command selected on the dashboard
    // NOTE: Don't override this, you'll forget to change it back during competition
    // Unless you're demoing, in which case you don't want to have an auto at all
    return new Noop();
  }
}
