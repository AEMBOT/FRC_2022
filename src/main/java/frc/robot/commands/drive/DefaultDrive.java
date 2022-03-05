package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;

public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;

  // Left & right stick inputs from main controller
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  //ramping constants
  private final double rampNew = 0.9;
  private final double rampOld = 1 - rampNew;

  // Drive at full speed for driver practice
  private double speedMultiplier = 1.0;

  //Deadzone, choose number from range (0,1)
  private double deadzone = 0.1;

  //Steering denominator
  private double steeringDenominator = 2;

  public DefaultDrive(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
    m_drive = drive;
    m_left = left;
    m_right = right;
    addRequirements(drive);
  }
double forwardPowerPrev = 0;
double rotationPowerPrev = 0;

  @Override
  public void execute() {
    // Log the powers to the dashboard
    double forwardPower = speedMultiplier * (MathUtil.applyDeadband(m_left.getAsDouble(), deadzone));
    forwardPower = forwardPower * rampOld + forwardPowerPrev * rampNew;
    forwardPowerPrev = forwardPower;
    SmartDashboard.putNumber("power", forwardPower);
    
    double rotationPower = speedMultiplier * ((MathUtil.applyDeadband(m_right.getAsDouble(), deadzone)) / steeringDenominator);
    rotationPower = rotationPower * rampOld + rotationPowerPrev + rampNew;
    SmartDashboard.putNumber("rotation", rotationPower);

    m_drive.arcadeDrive(forwardPower, rotationPower, true);
  }

}
