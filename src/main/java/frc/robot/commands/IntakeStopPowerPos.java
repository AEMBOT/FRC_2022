package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopPowerPos extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_subsystem;
  private double goalAngle;
  boolean shouldInvert; 

  public IntakeStopPowerPos(IntakeSubsystem subsystem,boolean invert, double angle){
    m_subsystem = subsystem;
    shouldInvert = invert; 
    goalAngle = angle; 
    addRequirements(subsystem);
  }



  @Override
  public void initialize(){
    if (m_subsystem.homingComplete == false) {

      new IntakeHoming(m_subsystem);
    }




    double motorAngle = m_subsystem.getLiftPosition();
    int direction = (shouldInvert ? -1 : 1);
    double power = 0.3 * direction;
    if(goalAngle > motorAngle){
      m_subsystem.setLiftPower(power);
    }

    if(goalAngle < motorAngle){
      power = -power; 
      m_subsystem.setLiftPower(power);
    }
  }

  @Override
  public void execute(){
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setLiftPower(0);
  }

  @Override
  public boolean isFinished() {
    double thresholdAngle = 8; 
    int direction = (shouldInvert ? -1 : 1);
    double power = 0.3 * direction; 
    if (Math.abs(goalAngle - m_subsystem.getLiftPosition()) > thresholdAngle){
      m_subsystem.setLiftPower(power);
    }
    else{
      m_subsystem.setLiftPower(0);
    }

    return (goalAngle <= m_subsystem.getLiftPosition());
  }
}
