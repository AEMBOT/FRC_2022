package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightTargeting;

import static frc.robot.Constants.DriveConstants.TurnPID.*;

import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;

public class TurnToSupplierAngle extends CommandBase {
  private DriveSubsystem m_drive;
  private double m_left;
  private double m_right;
  DoubleSupplier m_getAngle;

  /**
   * A command to drive a distance using Smart Motion on the Spark Maxes.
   *
   * @param angle The angle to turn to
   * @param drive The robot's DriveSubsystem
   */
  public TurnToSupplierAngle(DoubleSupplier angle, DriveSubsystem drive) {
    //TODO: make this a constant
    m_getAngle = angle;
    m_drive = drive;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    // Make sure motors are running in brake mode to avoid overshooting
    m_drive.setBrakeMode();

    double angle = m_getAngle.getAsDouble();
    m_left = angle * 0.0055;
    m_right = -m_left;

    // Reset the encoders & heading before moving
    m_drive.resetHeading();
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