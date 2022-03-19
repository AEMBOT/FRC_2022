package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeWinchToPosition extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;
  private double goalPosition;
  boolean shouldInvert; 

  public RunIntakeWinchToPosition(IntakeSubsystem intake, double position){
    m_intake = intake;
    goalPosition = position; 
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    double motorPosition = m_intake.getWinchPosition();
    if(goalPosition > motorPosition){
      m_intake.raiseIntake();
    }

    if(goalPosition < motorPosition){
      m_intake.lowerIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopWinch();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(goalPosition - m_intake.getWinchPosition()) < 
      Constants.IntakeConstants.kIntakeWinchMotionThreshold;
  }
}
