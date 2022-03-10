package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;

  // Left & right stick inputs from main controller
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  //ramping constants
  // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
  SlewRateLimiter forwardSlewLimiter = new SlewRateLimiter(1.0);
  SlewRateLimiter turningSlewLimiter = new SlewRateLimiter(1.0);

  // Drive at full speed for driver practice
  private double speedMultiplier = 1.0;
  private double rotationMultiplier = 0.6;

  //Deadzone, choose number from range (0,1)
  private double deadzone = 0.1;

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
    double forwardPower = speedMultiplier * (MathUtil.applyDeadband(-m_left.getAsDouble(), deadzone));
    // Slew filter
    forwardPower = forwardSlewLimiter.calculate(forwardPower);
    SmartDashboard.putNumber("power", forwardPower);
    
    double rotationPower = rotationMultiplier * (MathUtil.applyDeadband(-m_right.getAsDouble(), deadzone));
    rotationPower = turningSlewLimiter.calculate(rotationPower);
    SmartDashboard.putNumber("rotation", rotationPower);

    m_drive.arcadeDrive(forwardPower, rotationPower, true);
  }

  /*
    linear teleop profiling
    if accellerating
      while not at goal speed
        current speed += 0.2
        pass speed to drive control
    else (deccelerating)
      while not at goal speed
        current speed -= 0.2
        pass speed to drive control

    Notes: by using a slop of .02, the robot should thearetically take 1 seccond to reach full speed from 0
  */

}
