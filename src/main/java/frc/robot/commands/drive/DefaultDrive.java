package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
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
  // Creates a SlewRateLimiter that limits the rate of change of the signal to X.X units per second
  SlewRateLimiter forwardSlewLimiter = new SlewRateLimiter(2.0);
  SlewRateLimiter turningSlewLimiter = new SlewRateLimiter(2.0);

  // Drivers wanted quicker acceleration when playing defense
  SlewRateLimiter defenseSlewLimiter = new SlewRateLimiter(2.5);

  // Drive at full speed for driver practice
  private double speedMultiplier = 1.0;
  private double rotationMultiplier = 0.6;

  //Deadzone, choose number from range (0,1)
  // TODO: Move this to DriveSubsystem (using the setDeadband method)
  private double deadzone = 0.1;

  // Used to swap between faster/slower speeds
  BooleanSupplier m_changeMode;
  boolean defenseMode = false;

  public DefaultDrive(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right, BooleanSupplier changeMode) {
    m_drive = drive;
    m_left = left;
    m_right = right;
    m_changeMode = changeMode;
    addRequirements(drive);
  }

  double forwardPowerPrev = 0;
  double rotationPowerPrev = 0;

  @Override
  public void execute() {
    // FIXME: This will activate multiple times a button press, so this has to be implemented differently
    if (m_changeMode.getAsBoolean()) {
      defenseMode = !defenseMode;
    }

    // Log the powers to the dashboard
    double forwardPower = speedMultiplier * (MathUtil.applyDeadband(-m_left.getAsDouble(), deadzone));
    // Slew filter
    // if (defenseMode) {
    //   forwardPower = defenseSlewLimiter.calculate(forwardPower);
    // } else {
    //   forwardPower = forwardSlewLimiter.calculate(forwardPower);
    //   forwardPower = forwardSlewLimiter.calculate(forwardPower);
    // }
    // SmartDashboard.putNumber("power", forwardPower);
    forwardPower = defenseSlewLimiter.calculate(forwardPower);
    
    double rotationPower = rotationMultiplier * (MathUtil.applyDeadband(-m_right.getAsDouble(), deadzone));
    rotationPower = turningSlewLimiter.calculate(rotationPower);
    // SmartDashboard.putNumber("rotation", rotationPower);

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
