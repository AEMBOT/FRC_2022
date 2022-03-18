// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    // TODO: We might want to do this at a lower power
    m_intake.raiseIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopLift();

    if (!interrupted) {
      double max = m_intake.getLiftPosition();
      double min = max - Constants.IntakeConstants.kLiftRangeOfMotion;
      m_intake.setHome(min, max);
    }
  }

  @Override
  public boolean isFinished() {
    // Stop the command when the intake starts drawing too much current
    return m_intake.isAtHardLimit();
  }
}
