package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.TurnPID.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class TurnDegrees extends ProfiledPIDCommand {

  private static final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kS, kVDegreesPerSecond);
  private DriveSubsystem m_drive;

  /** Turns the specified number of degrees. */
  public TurnDegrees(double goalAngle, DriveSubsystem drive) {
    this(() -> goalAngle, drive);
  }

  /** Turns the number of degrees supplied by goalAngle when this command is run. */
  public TurnDegrees(DoubleSupplier goalAngle, DriveSubsystem drive) {
    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                kMaxVelocityDegreesPerSecond, kMaxAccelerationDegreesPerSecondSquared)),
        drive::getHeading,
        goalAngle,
        (output, setpoint) ->
            drive.arcadeDrive(0, output + m_feedforward.calculate(setpoint.velocity), false),
        drive);

    m_drive = drive;

    // Make gyro values wrap around to avoid taking the long route to an angle
    getController().enableContinuousInput(-180, 180);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);
  }

  @Override
  public void initialize() {
    // Make sure to reset the heading before resetting the internal PID controller
    m_drive.resetHeading();
    super.initialize();
  }

  @Override
  public void execute() {
    // SmartDashboard.putNumber("Setpoint Velocity", getController().getSetpoint().velocity);
    // SmartDashboard.putNumber(
    //     "Current Velocity",
    //     super.getController().getSetpoint().velocity + getController().getVelocityError());
    // SmartDashboard.putNumber(
    //     "Profiled turn power", m_feedforward.calculate(getController().getSetpoint().velocity));

    super.execute();
  }

  @Override
  public boolean isFinished() {
    // The command finishes once the robot is done turning
    return getController().atGoal();
  }
}
