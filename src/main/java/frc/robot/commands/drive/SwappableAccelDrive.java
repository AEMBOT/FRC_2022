package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Allows for swapping between two accelerations for defense and normal driving. */
public class SwappableAccelDrive extends CommandBase {
  private final DriveSubsystem m_drive;

  // Left & right stick inputs from main controller
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  // Acceleration limiters for forward/backward movement
  SlewRateLimiter m_forwardSlewLimiter = new SlewRateLimiter(2.0);
  SlewRateLimiter m_defenseSlewLimiter = new SlewRateLimiter(2.5);

  // Acceleration limiter for turning
  SlewRateLimiter m_rotationSlewLimiter = new SlewRateLimiter(2.0);

  // For acceleration swapping purposes
  double m_previousForwardPower = 0;

  // Used to swap between faster/slower speeds
  BooleanSupplier m_changeMode;
  boolean m_defenseMode = false;

  public SwappableAccelDrive(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
    m_drive = drive;
    m_left = left;
    m_right = right;
    addRequirements(drive);
  }

  /** Increases the allowed robot acceleration for defense purposes */
  public void toggleDefenseMode() {
    m_defenseMode = !m_defenseMode;

    // Reset the appropriate slew limiter
    if (m_defenseMode) {
      m_defenseSlewLimiter.reset(m_previousForwardPower);
    } else {
      m_forwardSlewLimiter.reset(m_previousForwardPower);
    }
  }

  @Override
  public void execute() {
    // Log the powers to the dashboard
    double forwardPower = kMaxForwardPower * -m_left.getAsDouble();

    // Use different slew limiters depending on whether "defense mode" is enabled
    if (m_defenseMode) {
      forwardPower = m_defenseSlewLimiter.calculate(forwardPower);
    } else {
      forwardPower = m_forwardSlewLimiter.calculate(forwardPower);
    }

    double rotationPower = kMaxRotationPower * -m_right.getAsDouble();
    rotationPower = m_rotationSlewLimiter.calculate(rotationPower);

    m_drive.arcadeDrive(forwardPower, rotationPower, true);
  }
}
