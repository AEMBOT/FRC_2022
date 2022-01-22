package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;

    public DefaultDrive(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
        m_drive = drive;
        m_left = left;
        m_right = right;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        m_drive.tankDrive(0.5 * m_left.getAsDouble(), 0.5 * m_right.getAsDouble());
    }
}
