package frc.robot.commands.utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that finishes immediately after doing nothing. */
public class Noop extends CommandBase {
  @Override
  public boolean isFinished() {
    return true;
  }
}
