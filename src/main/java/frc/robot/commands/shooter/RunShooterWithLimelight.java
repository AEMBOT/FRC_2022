package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterWithLimelight extends CommandBase {
  private ShooterSubsystem m_shooter;

  /** Runs the shooter at the correct RPM using the Limelight. */
  public RunShooterWithLimelight(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    // Run the shooter using the limelight, or at the set default RPM
    m_shooter.runAtCalibratedRPM();
  }

  @Override
  public void end(boolean _interrupted) {
    // Stop running the shooter after finishing
    m_shooter.stopShooter();
  }
}
