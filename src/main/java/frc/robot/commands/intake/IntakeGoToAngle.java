package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeGoToAngle extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;
  private double goalAngle;
  boolean shouldInvert; 

  public IntakeGoToAngle(IntakeSubsystem intake, double angle){
    m_intake = intake;
    goalAngle = angle; 
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    double motorAngle = m_intake.getLiftPosition();
    if(goalAngle > motorAngle){
      m_intake.setLiftPower(Constants.IntakeConstants.kNormalPower);
    }

    if(goalAngle < motorAngle){
      m_intake.setLiftPower(-Constants.IntakeConstants.kNormalPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setLiftPower(0);
  }

  @Override
  public boolean isFinished() {
    double thresholdAngle = 8; 
    if (Math.abs(goalAngle - m_intake.getLiftPosition()) < thresholdAngle){
      m_intake.setLiftPower(0);
    }
    else{
      
    }

    return (goalAngle <= m_intake.getLiftPosition());
  }
}
