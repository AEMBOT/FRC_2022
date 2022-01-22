package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
    private final DriveSubsystem m_drive;

    // Left & right stick inputs from main controller
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;

    // Don't drive at full speed for testing purposes
    private double speedMultiplier = 0.4;

    public DefaultDrive(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
        m_drive = drive;
        m_left = left;
        m_right = right;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Log the powers to the dashboard
        double forwardPower = speedMultiplier * m_left.getAsDouble();
        SmartDashboard.putNumber("power", forwardPower);
        double rotationPower = speedMultiplier * m_right.getAsDouble();
        SmartDashboard.putNumber("rotation", rotationPower);

        m_drive.arcadeDrive(forwardPower, rotationPower);
    }
}
