package frc.robot.commands.drive;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

/** A command that's responsible for driving of the robot during teleop. */
public class DefaultDrive extends CommandBase {
  private final DrivetrainSubsystem m_drive;

  // Left & right stick inputs from main controller
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_turning;

  // Acceleration limiter for forward/rotational movement
  SlewRateLimiter m_forwardSlewLimiter = new SlewRateLimiter(2.0);
  SlewRateLimiter m_turningSlewLimiter = new SlewRateLimiter(2.0);

  /**
   * Constructs a DefaultDrive command for driving the robot during teleop.
   *
   * @param drive The robot's drive subsystem
   * @param forwardPower The input for driving the robot forward/backward (normally left stick Y)
   * @param turningPower The input for turning the robot (normally right stick X)
   */
  public DefaultDrive(
      DrivetrainSubsystem drive, DoubleSupplier forwardPower, DoubleSupplier turningPower) {
    m_drive = drive;
    m_forward = forwardPower;
    m_turning = turningPower;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Deadband & scale the stick inputs
    double forwardPower =
        kMaxForwardPower * MathUtil.applyDeadband(-m_forward.getAsDouble(), kJoystickDeadband);
    double rotationPower =
        kMaxRotationPower * MathUtil.applyDeadband(m_turning.getAsDouble(), kJoystickDeadband);

    // Use slew limiters to limit the robot's acceleration
    forwardPower = m_forwardSlewLimiter.calculate(forwardPower);
    rotationPower = m_turningSlewLimiter.calculate(rotationPower);

    // Drive the robot
    m_drive.arcadeDrive(forwardPower, rotationPower, true);
  }
}
