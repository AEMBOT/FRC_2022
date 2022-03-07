package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimeRotation extends CommandBase {
  private DriveSubsystem m_drive;
  private double m_power;
  private final Timer m_timer = new Timer();

  public TimeRotation(double power, DriveSubsystem drive) {
    m_power = power;
    m_drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset the robot's heading
    m_drive.resetHeading();

    // Reset & start the timer to time the rotation
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_power, false);
    SmartDashboard.putNumber("Turn Time", m_timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the timer
    m_timer.stop();

    // Put the heading onto the dashboard in case the robot continues moving
    SmartDashboard.putNumber("Turn Amount", m_drive.getAngle());
  }

  @Override
  public boolean isFinished() {
    // Stop when the robot has rotated close to a full rotation
    return m_drive.getAngle() >= 355;
  }
}
