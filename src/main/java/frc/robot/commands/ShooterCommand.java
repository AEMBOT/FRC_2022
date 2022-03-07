package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterCommand extends CommandBase {
  private final ShooterSubsystem m_shooter;

  private final DoubleSupplier m_power;

  public ShooterCommand(ShooterSubsystem shooter, DoubleSupplier powerSupplier) {
    m_shooter = shooter;
    m_power = powerSupplier;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    m_shooter.runShooter(m_power.getAsDouble());
  }
}
