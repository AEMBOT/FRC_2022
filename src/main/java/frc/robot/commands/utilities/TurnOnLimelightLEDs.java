package frc.robot.commands.utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.LEDMode;

public class TurnOnLimelight extends CommandBase {
    private Limelight m_limelight;

    public TurnOnLimelight(Limelight limelight) {
        m_limelight = limelight;
    }

    @Override
    public void execute() {
        m_limelight.setLEDMode(LEDMode.On);
    }
}
