package frc.robot.commands.drive;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;

  // Left & right stick inputs from main controller
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  // Acceleration limiter for forward/rotational movement
  SlewRateLimiter m_forwardSlewLimiter = new SlewRateLimiter(2.0);
  SlewRateLimiter m_turningSlewLimiter = new SlewRateLimiter(2.0);

  public DefaultDrive(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
    m_drive = drive;
    m_left = left;
    m_right = right;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Calculate appropriate powers using above slew limiters
    double forwardPower =
        kMaxForwardPower * MathUtil.applyDeadband(-m_left.getAsDouble(), kJoystickDeadband);
    forwardPower = m_forwardSlewLimiter.calculate(forwardPower);

    double rotationPower =
        kMaxRotationPower * MathUtil.applyDeadband(-m_right.getAsDouble(), kJoystickDeadband);
    rotationPower = m_turningSlewLimiter.calculate(rotationPower);

    m_drive.arcadeDrive(forwardPower, rotationPower, true);
  }
}
