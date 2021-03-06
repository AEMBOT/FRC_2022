package frc.robot.commands.utilities;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/** A command that times how long it takes for the robot to rotate ~360 degrees. */
public class TimeRotation extends CommandBase {
  private DrivetrainSubsystem m_drive;
  private double m_power;
  private final Timer m_timer = new Timer();

  /**
   * Constructs a new TimeRotation command, which times how long it takes for the robot to rotate
   * ~360 degrees at a specific power.
   *
   * @param power The power to rotate the robot at
   * @param drive The robot's drive subsystem
   */
  public TimeRotation(double power, DrivetrainSubsystem drive) {
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
