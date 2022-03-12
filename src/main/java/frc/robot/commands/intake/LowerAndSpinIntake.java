package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class LowerAndSpinIntake extends CommandBase {
    private IntakeSubsystem m_intake;

    public LowerAndSpinIntake(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.lowerIntake();
        m_intake.runRollerAtMaxPower(false);
    }

    @Override
    public void end(boolean _interrupted) {
        // Stop the intake roller and bring it back up
        m_intake.setRPM(0);
        m_intake.raiseIntake();
    }
}
