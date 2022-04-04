package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.TurnPID.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

/** A command that turns the robot the specified number of degrees. */
public class TurnDegrees extends ProfiledPIDCommand {
  // Feedforward - converts angular velocities to power outputs
  private static final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kS, kVDegreesPerSecond);

  private DrivetrainSubsystem m_drive;

  /**
   * Turns the specified number of degrees.
   *
   * @param degrees The number of degrees to turn
   * @param drive The robot's drive subsystem
   */
  public TurnDegrees(double degrees, DrivetrainSubsystem drive) {
    this(() -> degrees, drive);
  }

  /**
   * Turns the number of degrees supplied by angleSupplier when this command is run.
   *
   * @param angleSupplier A method to run in order to obtain the angle to turn to
   * @param drive The robot's drive subsystem
   */
  public TurnDegrees(DoubleSupplier angleSupplier, DrivetrainSubsystem drive) {
    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                kMaxVelocityDegreesPerSecond, kMaxAccelerationDegreesPerSecondSquared)),
        drive::getAngle,
        angleSupplier,
        (output, setpoint) ->
            drive.arcadeDrive(0, output + m_feedforward.calculate(setpoint.velocity), false),
        drive);

    m_drive = drive;

    // Set the controller position & velocity tolerances
    m_controller.setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);
  }

  @Override
  public void initialize() {
    // Make sure to reset the heading before resetting the internal PID controller
    m_drive.resetHeading();
    super.initialize();
  }

  @Override
  public boolean isFinished() {
    // The command finishes once the robot is done turning
    return m_controller.atGoal();
  }
}
