package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RampShooter extends CommandBase {
  private ShooterSubsystem m_shooter;
  private double m_targetRPM;

  public RampShooter(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_targetRPM = m_shooter.getTargetRPM();
  }

  @Override
  public void execute() {
    m_shooter.runAtCalibratedRPM();
  }

  @Override
  public boolean isFinished() {
    return m_shooter.getFlywheelRPM() >= m_targetRPM - kRPMTolerance;
  }
}
