package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
// import java.util.function.DoubleSupplier;

public class RunShooterForTime extends CommandBase {
  private ShooterSubsystem m_shooter;

  private double m_duration;
  private Timer m_timer;
  // private DoubleSupplier m_power;

  public RunShooterForTime(double duration, ShooterSubsystem shooter) {
    m_duration = duration;
    m_shooter = shooter;
    addRequirements(shooter);
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    m_timer.start();
  }

  @Override
  public void execute() {
    m_shooter.runAtMapRPM();
  }

  @Override
  public void end(boolean _interrupted) {
    m_shooter.stopShooter();
    m_timer.stop();
    m_timer.reset();
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > m_duration;
  }
}
