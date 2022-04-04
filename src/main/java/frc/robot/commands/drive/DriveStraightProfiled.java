package frc.robot.commands.drive;

import static frc.robot.Constants.DrivetrainConstants.StraightPID.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A command that uses PID control, feedforward, and motion profiling to drive straight.
 *
 * <p>This hasn't been tuned recently, but it's kept around for reference.
 */
public class DriveStraightProfiled extends ProfiledPIDCommand {
  // Feedforward for the drive motors
  private static SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kS, kVSecondsPerMeter);

  /**
   * Constructs a DriveStraightProfiled command to drive forward some distance by following a motion
   * profile.
   *
   * @param distance The distance to drive
   * @param drive The robot's drive subsystem
   */
  public DriveStraightProfiled(double distance, DrivetrainSubsystem drive) {
    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                kMaxVelocityMetersPerSecond, kMaxAccelerationMeterPerSecondSquared)),
        drive::getLeftEncoderPosition,
        distance,
        (output, setpoint) ->
            drive.arcadeDrive(output + m_feedforward.calculate(setpoint.velocity), 0, false),
        drive);

    m_controller.setTolerance(kDriveToleranceMeters, kDriveVelocityToleranceMetersPerSecond);
  }

  @Override
  public boolean isFinished() {
    return m_controller.atGoal();
  }
}
