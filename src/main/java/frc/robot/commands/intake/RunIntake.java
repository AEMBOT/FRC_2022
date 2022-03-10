package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
    private IntakeSubsystem m_intake;

    public RunIntake(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.runRollerAtMaxPower();
    }

    @Override
    public void end(boolean _interrupted) {
        m_intake.setRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
