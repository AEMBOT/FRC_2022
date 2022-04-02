package frc.robot.commands.drive;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * A command that allows for swapping between two accelerations for defense and normal driving
 * during teleop.
 */
public class SwappableAccelDrive extends CommandBase {
  private DriveSubsystem m_drive;

  // Left & right stick inputs from main controller
  private DoubleSupplier m_forwardInput;
  private DoubleSupplier m_turningInput;

  // Boolean switch for enabling defense mode
  private DoubleSupplier m_defenseToggle;

  // Acceleration limiters for forward/backward movement
  private final SlewRateLimiter m_scoringSlewLimiter = new SlewRateLimiter(2.0);
  private final SlewRateLimiter m_defenseSlewLimiter = new SlewRateLimiter(2.5);

  // Acceleration limiter for turning
  private final SlewRateLimiter m_turningSlewLimiter = new SlewRateLimiter(2.0);

  // For acceleration swapping purposes
  private double m_previousForwardPower = 0;

  // Used to swap between faster/slower accelerations
  private boolean m_inDefenseMode = false;

  /**
   * Constructs a SwappableAccellDrive command, which allows for swapping beteen accelerations while
   * driving the robot during teleop.
   *
   * @param drive The robot's drive subsystem
   * @param forward The input for driving the robot forward/backward (normally left stick Y)
   * @param turning The input for turning the robot (normally right stick X)
   * @param defenseToggle The input for toggling defense mode when held (normally right trigger)
   */
  public SwappableAccelDrive(
      DriveSubsystem drive,
      DoubleSupplier forward,
      DoubleSupplier turning,
      DoubleSupplier defenseToggle) {
    m_drive = drive;
    m_forwardInput = forward;
    m_turningInput = turning;
    m_defenseToggle = defenseToggle;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Calculate the deadbanded and scaled forward/rotation powers
    double forwardPower =
        kMaxForwardPower * MathUtil.applyDeadband(-m_forwardInput.getAsDouble(), kJoystickDeadband);
    double rotationPower =
        kMaxRotationPower * MathUtil.applyDeadband(m_turningInput.getAsDouble(), kJoystickDeadband);

    // Apply the slew (acceleration) limiters to avoid burning the carpet/etc.
    rotationPower = m_turningSlewLimiter.calculate(rotationPower);

    // Slew limiters need to be reset if we're just entering/exiting defense mode

    // Entering defense mode
    if (m_defenseToggle.getAsDouble() >= kTriggerActiveThreshold && !m_inDefenseMode) {
      m_inDefenseMode = true;
      m_defenseSlewLimiter.reset(m_previousForwardPower);
    }

    // Exiting defense mode
    else if (m_defenseToggle.getAsDouble() < kTriggerActiveThreshold && m_inDefenseMode) {
      m_inDefenseMode = false;
      m_scoringSlewLimiter.reset(m_previousForwardPower);
    }

    // Apply the appropriate slew limiter to the stick input for forward/backward motion
    if (m_inDefenseMode) {
      forwardPower = m_defenseSlewLimiter.calculate(forwardPower);
    } else {
      forwardPower = m_scoringSlewLimiter.calculate(forwardPower);
    }

    // Actually drive the robot
    m_drive.arcadeDrive(forwardPower, rotationPower, true);

    // Used for slew limiter swapping
    m_previousForwardPower = forwardPower;
  }
}
