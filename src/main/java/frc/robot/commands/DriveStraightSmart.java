package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightSmart extends CommandBase {
  private DriveSubsystem m_drive;
  private double m_distance;

  public DriveStraightSmart(double distance, DriveSubsystem drive) {
    m_distance = distance;
    m_drive = drive;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    // Reset the heading and encoders
    m_drive.setBrakeMode();
    m_drive.resetHeading();
  }

  @Override
  public void execute() {
    m_drive.feedDrive();
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
