package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.IntakeStopPowerPos;

public class IntakeUprightPosition extends CommandBase {
  private final IntakeSubsystem m_subsystem;

  public IntakeUprightPosition(IntakeSubsystem subsystem){
    m_subsystem = subsystem;
  }

  @Override
  public void initialize(){}

  @Override
  public void execute() {
    new IntakeStopPowerPos(m_subsystem,false, 90);
  }

  @Override
  public void end(boolean interrupted){
    m_subsystem.setLiftPower(0);
  }
}

