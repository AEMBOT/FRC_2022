package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.TurnPID.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightTargeting;

public class AlignWithHub extends ProfiledPIDCommand {
  private static final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kSVolts, kVVoltDegreesPerSecond);
  private DriveSubsystem m_drive;
  private LimeLightTargeting m_limelight;
  private final Timer m_startupTimer = new Timer();

  public AlignWithHub(DriveSubsystem drive, LimeLightTargeting limelight) {
    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                kMaxVelocityDegreesPerSecond, kMaxAccelerationDegreesPerSecondSquared)),
        limelight::getX,
        0,
        (output, setpoint) ->
            drive.arcadeDrive(0, output + m_feedforward.calculate(setpoint.velocity), false),
        drive);

    m_drive = drive;
    m_limelight = limelight;

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    m_controller.setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);
  }

  @Override
  public void initialize() {
    // Make sure to reset the heading before resetting the internal PID controller
    m_drive.resetHeading();
    m_limelight.turnOnLED();
    m_startupTimer.start();
    super.initialize();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Setpoint Velocity", getController().getSetpoint().velocity);
    SmartDashboard.putNumber(
        "Current Velocity",
        super.getController().getSetpoint().velocity + getController().getVelocityError());
    SmartDashboard.putNumber(
        "Profiled turn power", m_feedforward.calculate(getController().getSetpoint().velocity));

    super.execute();
  }

  @Override
  public void end(boolean _interrupted) {
    m_limelight.turnOffLED();
    m_startupTimer.stop();
    m_startupTimer.reset();
  }

  @Override
  public boolean isFinished() {
    // The command finishes once the robot is done turning
    return m_startupTimer.get() > 0.2 && m_controller.atGoal();
  }
}
