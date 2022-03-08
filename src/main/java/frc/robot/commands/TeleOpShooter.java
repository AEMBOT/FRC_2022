package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
// import java.util.function.DoubleSupplier;

public class TeleOpShooter extends CommandBase {
  private ShooterSubsystem m_shooter;
  // private DoubleSupplier m_power;

  public TeleOpShooter(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute() {
    m_shooter.runAtMapRPM();
  }

  @Override
  public void end(boolean _interrupted) {
    m_shooter.stopShooter();
  }

}