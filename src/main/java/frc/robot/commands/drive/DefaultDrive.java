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
  private double rotationMultiplier = 0.7;

  //Deadzone, choose number from range (0,1)
  private double deadzone = 0.05;

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
    forwardPower = forwardPower * rampNew + forwardPowerPrev * rampOld;
    forwardPowerPrev = forwardPower;
    SmartDashboard.putNumber("power", forwardPower);
    
    double rotationPower = rotationMultiplier * (MathUtil.applyDeadband(m_right.getAsDouble(), deadzone));
    SmartDashboard.putNumber("rotation", rotationPower);

    // For whatever reason it was driving backwards. Weird.
    m_drive.arcadeDrive(forwardPower, -rotationPower, true);
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
