package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class LogGyroData extends CommandBase {
    private DriveSubsystem m_drive;

    public LogGyroData(DriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public void execute() {
        AHRS ahrs = m_drive.getAhrs();
        SmartDashboard.putNumber("yaw", ahrs.getYaw());
        SmartDashboard.putNumber("pitch", ahrs.getPitch());
        SmartDashboard.putNumber("roll", ahrs.getRoll());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
