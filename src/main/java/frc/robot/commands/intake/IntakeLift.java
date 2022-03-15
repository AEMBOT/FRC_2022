package frc.robot.commands.intake;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;

public class IntakeLift extends CommandBase {
  private IntakeSubsystem m_intake;
  private BooleanSupplier raise, lower;

  // Slew rate limiter for avoiding the intake motors becoming misaligned
  private final SlewRateLimiter m_rateLimiter = new SlewRateLimiter(2);

  private final double m_powerMultiplier = 0.5;

  public IntakeLift(IntakeSubsystem intake, BooleanSupplier raise, BooleanSupplier lower) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    double power;
    if (raise.getAsBoolean()) {
      power = m_rateLimiter.calculate(m_powerMultiplier);
    } else if (lower.getAsBoolean()) {
      power = m_rateLimiter.calculate(m_powerMultiplier);
    }

    // When nothing is pressed, eventually stop the lift
    else {
      power = m_rateLimiter.calculate(0);
    }

    m_intake.setLiftPower(power);
  }
}
