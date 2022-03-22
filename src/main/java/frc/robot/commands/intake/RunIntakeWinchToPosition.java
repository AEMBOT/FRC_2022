package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeWinchToPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;

  private double goalPosition;
  boolean shouldInvert;

  public RunIntakeWinchToPosition(IntakeSubsystem intake, double position) {
    m_intake = intake;
    goalPosition = position;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    double motorPosition = m_intake.getWinchPosition();
    if (goalPosition > motorPosition) {
      m_intake.raiseIntake();
    }

    if (goalPosition < motorPosition) {
      m_intake.lowerIntake();
    }
  }

  

  @Override
  public boolean isFinished() {
    boolean finished = false;
    if (Math.abs(goalPosition - m_intake.getWinchPosition())
        < Constants.IntakeConstants.kIntakeWinchMotionThreshold) {
      m_intake.stopWinch();
      // Default commands shouldn't end / be finished??
      //finished = true;
      finished = false;
    }
    return finished;
  }
}
