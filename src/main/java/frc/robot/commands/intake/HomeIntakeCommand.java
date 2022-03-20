// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** Homes the intake to its "max" point. */
public class HomeIntakeCommand extends CommandBase {
  private final IntakeSubsystem m_intake;

  /**
   * Creates a new HomeIntakeCommand.
   *
   * @param intake The subsystem used by this command.
   */
  public HomeIntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Raise intake until we hit hard limit
    m_intake.raiseIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopWinch();

    if (!interrupted) {
      m_intake.setHome();
    }
  }

  @Override
  public boolean isFinished() {
    // Stop the command when the intake starts drawing too much current
    return m_intake.isAtHardLimit();
  }
}
