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

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Invert the left motors to drive in the correct direction
        m_leftMotors.setInverted(true);

        // Reset the encoders & change their distance readings to meters
        setupEncoders();
        
        // Initialize the tracking of the robot's position on the field
        m_odometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()));
    }

    @Override
    public void periodic() {
        // Keep track of where the robot is on the field
        updateOdometry();

        // Log drive-related informatin to SmartDashboard if specified
        if (m_debug) {
            SmartDashboard.putNumber("Robot Heading", getHeading());
            SmartDashboard.putNumber("Rotation Velocity", m_ahrs.getVelocityZ());

            SmartDashboard.putNumber("Left Velocity", m_centerLeftEncoder.getVelocity());
            SmartDashboard.putNumber("Right Velocity", m_centerRightEncoder.getVelocity());

            // TODO: Figure out how to log motor voltages
        }
    }

    /**
     * Tank-style drive of the robot.
     * @param left The power to run the left motors at
     * @param right The power to run the right motors at
     */
    public void tankDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    /**
     * Arcade-style drive of the robot.
     * @param speed The forward-backward speed to run the motors at
     * @param rotation The rotation speed to run the motors at
     */
    public void arcadeDrive(double speed, double rotation) {
        m_drive.arcadeDrive(speed, rotation);
    }

    /** Gets the position of the center left encoder in meters. */
    public double getLeftEncoderPosition() {
        return m_centerLeftEncoder.getPosition();
    }

    /** Gets the position of the center right encoder in meters. */
    public double getRightEncoderPosition() {
        return m_centerRightEncoder.getPosition();
    }

    /** Resets the center motor encoders on each side & changes their output unit to inches */
    public void setupEncoders() {
        // Reset the motor encoders
        m_centerLeftEncoder.setPosition(0);
        m_centerRightEncoder.setPosition(0);

        // Set encoders to output measurements in meters (converted from rotations)
        m_centerLeftEncoder.setPositionConversionFactor(kWheelCircumferenceMeters);
        m_centerLeftEncoder.setVelocityConversionFactor(kWheelCircumferenceMeters);

        m_centerRightEncoder.setPositionConversionFactor(kWheelCircumferenceMeters);
        m_centerRightEncoder.setVelocityConversionFactor(kWheelCircumferenceMeters);
    }

    /** Gets the heading of the robot in degrees (-180 to 180). */
    public double getHeading() {
        return m_ahrs.getYaw();
    }

    /** Resets the robot heading to 0 degrees. */
    public void resetHeading() {
        m_ahrs.zeroYaw();
    }

    /**
     * Updates the DifferentialDriveOdometry instance variable.
     * It keeps track of where the robot is on the field, but can get thrown off
     * if the robot is bumped into/bumps into something.
     */
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
