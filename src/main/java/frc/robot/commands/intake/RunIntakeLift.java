// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utilities.enums.WinchDirection;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeLift extends CommandBase {
  private IntakeSubsystem m_intake;
  private WinchDirection m_direction;

  /**
   * Creates a new RunIntakeLift.
   *
   * @param intake The robot's {@link IntakeSubsystem} instance.
   */
  public RunIntakeLift(IntakeSubsystem intake, WinchDirection direction) {
    m_intake = intake;
    m_direction = direction;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    if (m_direction == WinchDirection.Up) {
      m_intake.raiseIntake();
    } else if (m_direction == WinchDirection.Down) {
      m_intake.lowerIntake();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean _interrupted) {
    m_intake.stopWinch();
  }

  @Override
  public boolean isFinished() {
    // Stop when the intake hits the robot or ground
    return m_intake.isAtHardLimit();
  }
}
