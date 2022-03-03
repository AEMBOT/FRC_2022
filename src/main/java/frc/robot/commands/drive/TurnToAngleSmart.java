package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.TurnPID.*;

public class TurnToAngleSmart extends CommandBase {
  private DriveSubsystem m_drive;
  private double m_left;
  private double m_right;

  /**
   * A command to drive a distance using Smart Motion on the Spark Maxes.
   *
   * @param angle The angle to turn to
   * @param drive The robot's DriveSubsystem
   */
  public TurnToAngleSmart(double angle, DriveSubsystem drive) {
    m_left = angle * kMetersPerDegree;
    m_right = -m_left;

    m_drive = drive;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    // Make sure motors are running in brake mode to avoid overshooting
    m_drive.setBrakeMode();

    // Reset the encoders & heading before moving
    m_drive.resetHeading();
  }

  @Override
  public void execute() {
    // Run the motors to the provided position
    m_drive.smartMotionToPosition(m_left, m_right);
  }

  @Override
  public boolean isFinished() {
    return m_drive.smartMotionAtGoal(m_left);
  }
}
