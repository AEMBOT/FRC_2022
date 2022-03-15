package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleOpShooter extends CommandBase {
  private ShooterSubsystem m_shooter;

  public TeleOpShooter(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    m_shooter.runAtCalibratedRPM();
  }

  @Override
  public void end(boolean _interrupted) {
    m_shooter.stopShooter();
  }
}
