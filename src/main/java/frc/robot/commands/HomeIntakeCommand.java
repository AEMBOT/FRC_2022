// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** Homes the intake to its "max" point. */
public class HomeIntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_subsystem;

  private final double kLiftRangeOfMotion = 50;

  /**
   * Creates a new HomeIntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HomeIntakeCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setLiftPower(.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      double max = m_subsystem.getLiftPosition();
      double min = max - kLiftRangeOfMotion;
      m_subsystem.setHome(min, max);
    }

    m_subsystem.setLiftPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isStopped = m_subsystem.isStopped();
    return isStopped;
  }
}
