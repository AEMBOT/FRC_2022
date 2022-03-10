package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RampShooter extends CommandBase {
    private ShooterSubsystem m_shooter;
    private double m_targetRPM;

    private NetworkTableEntry m_yAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");

    private final double m_RPMTolerance = 10;

    public RampShooter(ShooterSubsystem shooter) {
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        double ty = m_yAngle.getDouble(0.0);
        m_targetRPM = m_shooter.returnRPM(ty);
    }

    @Override
    public void execute() {
        m_shooter.runAtMapRPM();
    }

    @Override
    public boolean isFinished() {
        return m_shooter.getFlywheelRPM() >= m_targetRPM - m_RPMTolerance;
    }
}