package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
        new CANSparkMax(DriveConstants.kLeftFront, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kLeftCenter, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kLeftBack, MotorType.kBrushless)
    );
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
        new CANSparkMax(DriveConstants.kRightFront, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kRightCenter, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kRightBack, MotorType.kBrushless)
    );
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    public DriveSubsystem() {
        // Invert the left motors to drive in the correct direction
        m_leftMotors.setInverted(true);
    }

    public void tankDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }
}
