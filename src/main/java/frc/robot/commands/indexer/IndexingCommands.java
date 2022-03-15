// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// indexer logic using color sensor input

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utilities.TimedRumble;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexingCommands extends CommandBase {
  private final IndexerSubsystem m_index;
  private Alliance teamColor = Alliance.Invalid;
  private XboxController m_controller;
  private Alliance currentCargoColor = Alliance.Invalid;

  private Alliance positions[] = {Alliance.Invalid, Alliance.Invalid};

  public IndexingCommands(IndexerSubsystem subsystem, XboxController controller) {
    m_index = subsystem;
    teamColor = DriverStation.getAlliance();
    m_controller = controller;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (teamColor == Alliance.Invalid) {
      if (DriverStation.isFMSAttached()) {
        teamColor = DriverStation.getAlliance();
      } else {
        return; // Can't do anything unless we have a color
      }
    }

    currentCargoColor = m_index.getCargoAllianceColor();

    if (currentCargoColor == Alliance.Invalid) {
      // Nothing to do
      return;
    }

    if (currentCargoColor != teamColor) {
      // .25 seconds, .5 power
      new TimedRumble(m_controller, 0.25, .5);
    }

    // Always move up the cargo if we have a slot available, regardless of color
    if (positions[1] == Alliance.Invalid) {
      // Intake ball to 2nd position
      m_index.toggleExitSide(); // but need it to be on for a little bit, then turn off
    } else {
      // Reverse intake a little bit to remove any second balls immediately behind this one and
      // raise the intake?

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
