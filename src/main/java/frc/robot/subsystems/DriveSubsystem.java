package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // These motor controllers have to be instantiated separately for encoder purposes
    private final CANSparkMax m_centerLeftMotor = new CANSparkMax(DriveConstants.kLeftCenter, MotorType.kBrushless);
    private final CANSparkMax m_centerRightMotor = new CANSparkMax(DriveConstants.kRightCenter, MotorType.kBrushless);

    // Group together drive motors on the same side of the drivetrain (left/right)
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
        new CANSparkMax(DriveConstants.kLeftFront, MotorType.kBrushless),
        m_centerLeftMotor,
        new CANSparkMax(DriveConstants.kLeftBack, MotorType.kBrushless)
    );
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
        new CANSparkMax(DriveConstants.kRightFront, MotorType.kBrushless),
        m_centerRightMotor,
        new CANSparkMax(DriveConstants.kRightBack, MotorType.kBrushless)
    );

    // This allows us to read angle information from the NavX
    private final AHRS m_ahrs = new AHRS(SerialPort.Port.kMXP);

    // WPILib provides a convenience class for differential drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // This allows the robot to keep track of where it is on the field
    // private DifferentialDriveOdometry m_odometry;

    public DriveSubsystem() {
        // Invert the left motors to drive in the correct direction
        m_leftMotors.setInverted(true);

        // Reset the encoders after starting up the robot
        resetEncoders();
        
        // Initialize the robot's position on the field
        // TODO: Figure out a way to continously update this
        // m_odometry = new DifferentialDriveOdometry(getHeading());
    }

    public void tankDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    public void arcadeDrive(double speed, double rotation) {
        m_drive.arcadeDrive(speed, rotation);
    }

    public double getLeftEncoderPosition() {
        return m_centerLeftMotor.getEncoder().getPosition();
    }

    public double getRightEncoderPosition() {
        return m_centerRightMotor.getEncoder().getPosition();
    }

    // TODO: I don't know if this is actually how you should reset the encoders
    public void resetEncoders() {
        m_centerLeftMotor.getEncoder().setPosition(0);
        m_centerRightMotor.getEncoder().setPosition(0);
    }

    /**
     * Gets the heading of the robot
     * @return
     */
    public Rotation2d getHeading() {
        // Rotation2d requires that its angle be in radians
        double headingRadians = Math.toRadians(m_ahrs.getYaw());

        return new Rotation2d(headingRadians);
    }

    public AHRS getAhrs() {
        return m_ahrs;
    }
}
