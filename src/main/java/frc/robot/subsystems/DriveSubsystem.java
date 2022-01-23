package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {
    // These motor controllers have to be instantiated separately for encoder purposes
    private final CANSparkMax m_centerLeftMotor = new CANSparkMax(kLeftCenter, MotorType.kBrushless);
    private final CANSparkMax m_centerRightMotor = new CANSparkMax(kRightCenter, MotorType.kBrushless);

    private final RelativeEncoder m_centerLeftEncoder = m_centerLeftMotor.getEncoder();
    private final RelativeEncoder m_centerRightEncoder = m_centerRightMotor.getEncoder();

    // Group together drive motors on the same side of the drivetrain (left/right)
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
        new CANSparkMax(kLeftFront, MotorType.kBrushless),
        m_centerLeftMotor,
        new CANSparkMax(kLeftBack, MotorType.kBrushless)
    );
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
        new CANSparkMax(kRightFront, MotorType.kBrushless),
        m_centerRightMotor,
        new CANSparkMax(kRightBack, MotorType.kBrushless)
    );

    // This allows us to read angle information from the NavX
    private final AHRS m_ahrs = new AHRS(SerialPort.Port.kMXP);

    // WPILib provides a convenience class for differential drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // This allows the robot to keep track of where it is on the field
    private DifferentialDriveOdometry m_odometry;
    private double m_leftPosition = 0;
    private double m_rightPosition = 0;

    // Whether or not to log debug information to the SmartDashboard
    private final boolean m_debug = true;

    public DriveSubsystem() {
        // Invert the left motors to drive in the correct direction
        m_leftMotors.setInverted(true);

        // Reset the encoders & change their distance readings to meters
        setupEncoders();
        
        // Initialize the robot's position on the field
        m_odometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()));
    }

    @Override
    public void periodic() {
        // Keep track of where the robot is on the field
        updateOdometry();

        // Log drive-related informatin to SmartDashboard if specified
        if (m_debug) {
            SmartDashboard.putNumber("Robot Heading", getHeading());

            // TODO: Figure out how to log motor voltages
        }
    }

    public void tankDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    public void arcadeDrive(double speed, double rotation) {
        m_drive.arcadeDrive(speed, rotation);
    }

    public double getLeftEncoderPosition() {
        return m_centerLeftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return m_centerRightEncoder.getPosition();
    }

    // TODO: I don't know if this is actually how you should reset the encoders
    public void setupEncoders() {
        // Reset the motor encoders
        m_centerLeftEncoder.setPosition(0);
        m_centerRightEncoder.setPosition(0);

        // Set encoders to output measurements in meters (converted from rotations)
        m_centerLeftEncoder.setPositionConversionFactor(kWheelCircumferenceMeters);
        m_centerRightEncoder.setPositionConversionFactor(kWheelCircumferenceMeters);
    }

    /**
     * Gets the heading of the robot
     * @return
     */
    public double getHeading() {
        return m_ahrs.getYaw();
    }

    private void updateOdometry() {
        double currentLeftPosition = getLeftEncoderPosition();
        double currentRightPosition = getRightEncoderPosition();

        // Update the drive odometry
        m_odometry.update(
            new Rotation2d(getHeading()),
            currentLeftPosition - m_leftPosition,
            currentRightPosition - m_rightPosition
        );

        // Update previous encoder readings (odometry requires change in position)
        m_leftPosition = currentLeftPosition;
        m_rightPosition = currentRightPosition;
    }
}
