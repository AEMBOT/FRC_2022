package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/** A command that runs the shooter at the correct RPM based on what the Limelight sees. */
public class RunShooterWithLimelight extends CommandBase {
  private ShooterSubsystem m_shooter;

  /**
   * Constructs a RunShooterWithLimelight command, which runs the shooter at the correct RPM using
   * the Limelight.
   *
   * @param shooter The robot's shooter subsystem
   */
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
