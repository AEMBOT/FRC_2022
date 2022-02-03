package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(kLeftFront, MotorType.kBrushless);
  private final CANSparkMax m_centerLeftMotor = new CANSparkMax(kLeftCenter, MotorType.kBrushless);
  private final CANSparkMax m_backLeftMotor = new CANSparkMax(kLeftBack, MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(kRightFront, MotorType.kBrushless);
  private final CANSparkMax m_centerRightMotor =
      new CANSparkMax(kRightCenter, MotorType.kBrushless);
  private final CANSparkMax m_backRightMotor = new CANSparkMax(kRightBack, MotorType.kBrushless);

  private final RelativeEncoder m_centerLeftEncoder = m_centerLeftMotor.getEncoder();
  private final RelativeEncoder m_centerRightEncoder = m_centerRightMotor.getEncoder();

  // Group together drive motors on the same side of the drivetrain (left/right)
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_frontLeftMotor, m_centerLeftMotor, m_backLeftMotor);
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_frontRightMotor, m_centerRightMotor, m_backRightMotor);

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

    // m_frontLeftMotor.enableVoltageCompensation(nominalVoltage);
    // m_centerLeftMotor.enableVoltageCompensation(nominalVoltage);
    // m_backLeftMotor.enableVoltageCompensation(nominalVoltage);
    // m_frontRightMotor.enableVoltageCompensation(nominalVoltage);
    // m_centerRightMotor.enableVoltageCompensation(nominalVoltage);
    // m_backRightMotor.enableVoltageCompensation(nominalVoltage);

    // Reset the encoders & change their distance readings to meters
    resetEncoders();
    setupEncoderConversions();

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
      SmartDashboard.putNumber("Rotation Velocity", getRotationRate());

      SmartDashboard.putNumber("Left Velocity", m_centerLeftEncoder.getVelocity());
      SmartDashboard.putNumber("Right Velocity", m_centerRightEncoder.getVelocity());

      // TODO: Figure out how to log motor voltages
      SmartDashboard.putNumber("Left Power", m_leftMotors.get());
      SmartDashboard.putNumber("Right Power", m_rightMotors.get());
    }
  }

  /**
   * Tank-style drive of the robot.
   *
   * @param left the power to run the left motors at
   * @param right the power to run the right motors at
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Arcade-style drive of the robot.
   *
   * @param speed the forward-backward speed to run the motors at
   * @param rotation the rotation speed to run the motors at
   * @param squareInputs whether to "square" input parameter (magnitude only)
   */
  public void arcadeDrive(double speed, double rotation, boolean squareInputs) {
    m_drive.arcadeDrive(speed, rotation, squareInputs);
  }

  /** Gets the position of the center left encoder in meters. */
  private double getLeftEncoderPosition() {
    return m_centerLeftEncoder.getPosition();
  }

  /** Gets the position of the center right encoder in meters. */
  private double getRightEncoderPosition() {
    return m_centerRightEncoder.getPosition();
  }

  /** Resets the center drive motor encoders to a position of 0 */
  private void resetEncoders() {
    m_centerLeftEncoder.setPosition(0);
    m_centerRightEncoder.setPosition(0);
  }

  /**
   * Changes encoder readings to meters and meters per second for position and velocity,
   * respectively.
   */
  private void setupEncoderConversions() {
    // TODO: these multipliers don't seem to be very accurate, so the motors might be geared up/down
    m_centerLeftEncoder.setPositionConversionFactor(kWheelCircumferenceMeters);
    m_centerLeftEncoder.setVelocityConversionFactor(kWheelCircumferenceMeters);

    m_centerRightEncoder.setPositionConversionFactor(kWheelCircumferenceMeters);
    m_centerRightEncoder.setVelocityConversionFactor(kWheelCircumferenceMeters);
  }

  /** Gets the heading of the robot. Ranges from -180 to 180 degrees. */
  public double getHeading() {
    // Negated to mirror signs on a coordinate plane (+ -> counterclockwise and vice versa)
    return -m_ahrs.getYaw();
  }

  /**
   * Gets the angle the robot has rotated since the last gyro reset. Can be greater than 360
   * degrees.
   */
  public double getAngle() {
    return -m_ahrs.getAngle();
  }

  /** Gets the rate of change of the yaw of the robot. */
  public double getRotationRate() {
    // getRate only returns the difference in angles, so it has to be multiplied by the NavX update
    // frequency
    // (see https://github.com/kauailabs/navxmxp/issues/69)
    return -m_ahrs.getRate() * m_ahrs.getActualUpdateRate();
  }

  /** Resets the robot heading to 0 degrees, as well as the odometry. */
  public void resetHeading() {
    // TODO: This fails if the NavX is calibrating, so it might be a good idea to check for that
    m_ahrs.zeroYaw();

    // Resetting the odometry also requires zeroing the encoders
    resetEncoders();

    // Reuse the previous pose on the field, compensating for the change in heading
    Pose2d prev_pose = m_odometry.getPoseMeters();
    m_odometry.resetPosition(prev_pose, new Rotation2d(Units.degreesToRadians(getHeading())));
  }

  /**
   * Updates the DifferentialDriveOdometry instance variable. It keeps track of where the robot is
   * on the field, but can get thrown off if the robot is bumped into/bumps into something.
   */
  private void updateOdometry() {
    double currentLeftPosition = getLeftEncoderPosition();
    double currentRightPosition = getRightEncoderPosition();

    // Update the drive odometry
    m_odometry.update(
        new Rotation2d(Units.degreesToRadians(getHeading())),
        currentLeftPosition - m_leftPosition,
        currentRightPosition - m_rightPosition);

    // Update previous encoder readings (odometry requires change in position)
    m_leftPosition = currentLeftPosition;
    m_rightPosition = currentRightPosition;
  }
}
