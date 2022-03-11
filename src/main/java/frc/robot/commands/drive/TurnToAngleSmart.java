package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.TurnPID.*;

import java.util.function.DoubleSupplier;

public class TurnToAngleSmart extends CommandBase {
  private DriveSubsystem m_drive;
  private DoubleSupplier m_getAngle;
  private double m_left;
  private double m_right;

  /**
   * A command to drive a distance using Smart Motion on the Spark Maxes.
   *
   * @param getAngle The method that supplies the goal angle
   * @param drive The robot's DriveSubsystem
   */
  public TurnToAngleSmart(DoubleSupplier getAngle, DriveSubsystem drive) {
    m_getAngle = getAngle;
    m_drive = drive;
    addRequirements(m_drive);
  }

  /**
   * A command to drive a distance using Smart Motion on the Spark Maxes.
   *
   * @param angle The angle to turn to
   * @param drive The robot's DriveSubsystem
   */
  public TurnToAngleSmart(double angle, DriveSubsystem drive) {
    this(() -> angle, drive);
  }

  @Override
  public void initialize() {
    // Make sure motors are running in brake mode to avoid overshooting
    m_drive.setBrakeMode();

    // Reset the encoders & heading before moving
    m_drive.resetHeading();

    // Set the target positions for the motors from the supplier method
    double angle = m_getAngle.getAsDouble();
    m_left = angle * kMetersPerDegree;
    m_right = -m_left;
  }

  @Override
  public void execute() {
    // Run the motors to the provided position
    m_drive.smartMotionToPosition(m_left, m_right);
  }

  @Override
  public void end(boolean _interrupted) {
    m_drive.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return m_drive.smartMotionAtGoal(m_left);
  }
}
