package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

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
    m_distance = distance;
    m_drive = drive;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    // Make sure motors are running in brake mode
    m_drive.setBrakeMode();

    // Reset the encoders before running the motors
    m_drive.resetEncoders();
  }

  @Override
  public void execute() {
    // Run the motors to the provided position
    m_drive.smartMotionToPosition(m_distance);

    // Debugging info
    SmartDashboard.putNumber("Set Point", m_distance);
    SmartDashboard.putNumber("Left Position", m_drive.getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Position", m_drive.getRightEncoderPosition());
  }

  @Override
  public boolean isFinished() {
    return m_drive.smartMotionAtGoal(m_distance);
  }
}
