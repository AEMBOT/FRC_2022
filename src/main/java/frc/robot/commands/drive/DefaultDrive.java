package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;

  // Left & right stick inputs from main controller
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  // Drive at full speed for driver practice
  private double speedMultiplier = 1.0;

  //Deadzone, choose number from range (0,1)
  private double deadzone = 0.4;

  public DefaultDrive(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
    m_drive = drive;
    m_left = left;
    m_right = right;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Log the powers to the dashboard
    SmartDashboard.putNumber("LEFT", m_left.getAsDouble() );
    SmartDashboard.putNumber("RIGHT", m_right.getAsDouble() );

    double forwardPower = speedMultiplier * leftSitck();
    SmartDashboard.putNumber("power", forwardPower);
    double rotationPower = speedMultiplier * rightSitck();
    SmartDashboard.putNumber("rotation", rotationPower);

    m_drive.arcadeDrive(forwardPower, rotationPower, true);
  }

  //establish deadzones
  public double leftSitck() {
    if (Math.abs(m_left.getAsDouble()) > deadzone){
      return ((m_left.getAsDouble() - deadzone) * 1.6);
    }
    else {
      return 0;
    }
  }
  //establish deadzones
  public double rightSitck() {
    if (Math.abs(m_right.getAsDouble()) > deadzone){
      return -((m_right.getAsDouble() - deadzone) * 1.6);
    }
    else {
      return 0;
    }
  }

}
