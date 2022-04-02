package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command that uses Spark Max Smart Motion control (essentially motion profiling) to drive a
 * distance.
 */
public class DriveStraightSmart extends CommandBase {
  private DriveSubsystem m_drive;
  private double m_distance;

  /**
   * A command to drive a distance using Smart Motion on the Spark Maxes.
   *
   * @param distance The distance to drive
   * @param drive The robot's DriveSubsystem
   */
  public DriveStraightSmart(double distance, DriveSubsystem drive) {
    // TODO: Test that the distance no longer needs to be negated
    m_distance = distance;
    m_drive = drive;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    // Make sure motors are running in brake mode to avoid overshooting
    m_drive.setBrakeMode();

    // Reset the encoders (and odometry) before running the motors
    m_drive.resetOdometryAndEncoders();
  }

  @Override
  public void execute() {
    // Run the motors to the provided position
    m_drive.smartMotionToPosition(m_distance);
  }

  @Override
  public boolean isFinished() {
    return m_drive.smartMotionAtGoal(m_distance);
  }
}
