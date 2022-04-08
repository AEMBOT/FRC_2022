package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.DrivetrainConstants.StraightPID.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A subsystem that includes the robot's drive motors/encoders, as well as the NavX IMU. */
public class DrivetrainSubsystem extends SubsystemBase {
  // Drive motors
  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(kLeftFront, MotorType.kBrushless);
  private final CANSparkMax m_centerLeftMotor = new CANSparkMax(kLeftCenter, MotorType.kBrushless);
  private final CANSparkMax m_backLeftMotor = new CANSparkMax(kLeftBack, MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(kRightFront, MotorType.kBrushless);
  private final CANSparkMax m_centerRightMotor =
      new CANSparkMax(kRightCenter, MotorType.kBrushless);
  private final CANSparkMax m_backRightMotor = new CANSparkMax(kRightBack, MotorType.kBrushless);

  // Encoders for "center" motors (motor positions aren't really meaningful with gearbox)
  private final RelativeEncoder m_centerLeftEncoder = m_centerLeftMotor.getEncoder();
  private final RelativeEncoder m_centerRightEncoder = m_centerRightMotor.getEncoder();

  // Internal PID controllers for center motors
  private final SparkMaxPIDController m_leftController = m_centerLeftMotor.getPIDController();
  private final SparkMaxPIDController m_rightController = m_centerRightMotor.getPIDController();

  // Feedforward for drive motors
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kSVolts, kVSecondsPerMeter, kASecondsSquaredPerMeter);

  // Used to approximate acceleration for feedforward purposes
  private double m_previousLeftVelocity = 0;
  private double m_previousRightVelocity = 0;

  // This allows us to read angle information from the NavX
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP);

  // WPILib provides a convenience class for differential drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_centerLeftMotor, m_centerRightMotor);

  // This allows the robot to keep track of where it is on the field
  private final DifferentialDriveOdometry m_odometry;

  // Used to display the current robot position
  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem, configuring its motors, encoders, and odometry. */
  public DrivetrainSubsystem() {
    // Restore the default settings for all of the motors
    restoreMotorFactoryDefaults();

    // Invert the left motors to drive in the correct direction
    m_centerRightMotor.setInverted(true);

    // Have the front/back motors mirror power outputs from the center motors
    m_frontLeftMotor.follow(m_centerLeftMotor);
    m_backLeftMotor.follow(m_centerLeftMotor);

    m_frontRightMotor.follow(m_centerRightMotor);
    m_backRightMotor.follow(m_centerRightMotor);

    // Set current limits for all of the drive motors
    setCurrentLimits();

    // Set the SmartMotion constants for the center motors
    configureSparkMax(m_leftController);
    configureSparkMax(m_rightController);

    // Change encoder position readings to meters (per second)
    setupEncoderConversions();

    // Initialize the odometry to track where the robot is on the field
    m_odometry = new DifferentialDriveOdometry(m_navx.getRotation2d());

    // Display the robot's position on a field widget on the dashboard
    SmartDashboard.putData(m_field);
  }

  @Override
  public void periodic() {
    // Keep track of where the robot is on the field
    updateOdometry();

    // Update the field widget on the dashboard with the robot's current pose
    m_field.setRobotPose(getPose());
  }

  // MOTOR CONFIGURATION

  /** Set all drive motors to brake mode. */
  public void setBrakeMode() {
    m_frontLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_centerLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_backLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_frontRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_centerRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_backRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /** Set all drive motors to coast mode. */
  public void setCoastMode() {
    m_frontLeftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_centerLeftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_backLeftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_frontRightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_centerRightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_backRightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  /** Resets drive Spark Max configurations to their factory defaults */
  private void restoreMotorFactoryDefaults() {
    m_backLeftMotor.restoreFactoryDefaults();
    m_centerLeftMotor.restoreFactoryDefaults();
    m_frontLeftMotor.restoreFactoryDefaults();

    m_backRightMotor.restoreFactoryDefaults();
    m_centerRightMotor.restoreFactoryDefaults();
    m_frontRightMotor.restoreFactoryDefaults();
  }

  /** Sets smart current limits on all of the drive motor Spark Maxes. */
  private void setCurrentLimits() {
    m_backLeftMotor.setSmartCurrentLimit(60);
    m_centerLeftMotor.setSmartCurrentLimit(60);
    m_frontLeftMotor.setSmartCurrentLimit(60);

    m_backRightMotor.setSmartCurrentLimit(60);
    m_centerRightMotor.setSmartCurrentLimit(60);
    m_frontRightMotor.setSmartCurrentLimit(60);
  }

  /**
   * Sets various Smart Motion constants for a Spark Max PID controller.
   *
   * @param controller The {@link SparkMaxPIDController} to configure
   */
  private void configureSparkMax(SparkMaxPIDController controller) {
    int slot = 0;

    // PID/feedforward constants
    controller.setP(kP, slot);
    controller.setI(kI, slot);
    controller.setD(kD, slot);
    controller.setFF(kFF, slot);

    // Output range (might not actually have to be set)
    controller.setOutputRange(kMinOutput, kMaxOutput, slot);
  }

  // BASIC DRIVE METHODS

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

  /**
   * Tank drive of the robot using voltages supplied to the drive motors.
   *
   * @param leftVolts Voltage to apply to the left motors
   * @param rightVolts Voltage to apply to the right motors
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_centerLeftMotor.setVoltage(leftVolts);
    m_centerRightMotor.setVoltage(rightVolts);

    // Feed DifferentialDrive timer so it doesn't complain
    m_drive.feedWatchdog();
  }

  /**
   * Drives the left and right wheels at the specified velocities using closed loop velocity control
   * on the Spark Maxes.
   *
   * @param leftVelocity The velocity of the left wheels, in meters per second.
   * @param rightVelocity The velocity of the right wheels, in meters per second.
   */
  public void tankDriveVelocities(double leftVelocity, double rightVelocity) {
    // Calculate feedforward voltages for both sides, approximating acceleration by using 0.02 (the
    // default TimedRobot period) as the change in time
    double leftFeedforward =
        m_feedforward.calculate(leftVelocity, (leftVelocity - m_previousLeftVelocity) / 0.02);
    double rightFeedforward =
        m_feedforward.calculate(rightVelocity, (rightVelocity - m_previousRightVelocity) / 0.02);

    // Command the motors at the above feedforward voltages, using PID to correct for error
    m_leftController.setReference(
        leftVelocity, CANSparkMax.ControlType.kVelocity, 0, leftFeedforward);
    m_rightController.setReference(
        rightVelocity, CANSparkMax.ControlType.kVelocity, 0, rightFeedforward);

    // Feed the DifferentialDrive motor safety timer so it doesn't complain
    m_drive.feedWatchdog();

    // Update previous velocities
    m_previousLeftVelocity = leftVelocity;
    m_previousRightVelocity = rightVelocity;
  }

  /**
   * Resets the previous velocities used for feedforward calculations in {@link
   * #tankDriveVelocities(double, double)}.
   */
  public void resetPreviousVelocities() {
    m_previousLeftVelocity = 0;
    m_previousRightVelocity = 0;
  }

  /** Stops all drive motors. */
  public void stopMotors() {
    m_centerLeftMotor.set(0);
    m_centerRightMotor.set(0);
  }

  // ENCODER METHODS

  /** Gets the position of the center left encoder in meters. */
  public double getLeftEncoderPosition() {
    return m_centerLeftEncoder.getPosition();
  }

  /** Gets the position of the center right encoder in meters. */
  public double getRightEncoderPosition() {
    return m_centerRightEncoder.getPosition();
  }

  /**
   * Returns the current encoder velocities as a {@link DifferentialDriveWheelSpeeds} object for use
   * in a RAMSETE controller.
   *
   * @return The current left and right wheel speeds in meters per second
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_centerLeftEncoder.getVelocity(), m_centerRightEncoder.getVelocity());
  }

  /**
   * Changes encoder readings to meters and meters per second for position and velocity,
   * respectively.
   */
  private void setupEncoderConversions() {
    // Convert revolutions to meters
    m_centerLeftEncoder.setPositionConversionFactor(kMetersPerMotorRotation);
    m_centerRightEncoder.setPositionConversionFactor(kMetersPerMotorRotation);

    // Convert RPM to meters per second
    m_centerLeftEncoder.setVelocityConversionFactor(kRPMToMetersPerSecond);
    m_centerRightEncoder.setVelocityConversionFactor(kRPMToMetersPerSecond);
  }

  // NAVX METHODS

  /**
   * Gets the heading of the robot, with positive angles representing counterclockwise rotation.
   * Ranges from -180 to 180 degrees.
   */
  public double getHeading() {
    // Negated to mirror signs on a coordinate plane (+ -> counterclockwise and vice
    // versa)
    return -m_navx.getYaw();
  }

  /**
   * Gets the angle the robot has rotated since the last gyro reset. Can be greater than 360
   * degrees.
   *
   * <p>Positive angles represent counterclockwise rotation, as most WPILib classes expect.
   */
  public double getAngle() {
    return -m_navx.getAngle();
  }

  /** Resets the robot heading to 0 degrees, as well as the drive motor encoders and odometry. */
  public void resetHeading() {
    m_navx.zeroYaw();

    // Reset the odometry, reusing the previous pose on the field
    resetOdometryAndEncoders();
  }

  // ODOMETRY METHODS

  /** Returns the robot's current pose on the field in meters. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Resets the odometry and encoders while maintaining the same pose. */
  public void resetOdometryAndEncoders() {
    resetOdometryAndEncoders(getPose());
  }

  /**
   * Resets the odometry with the provided pose, also resetting the drive encoders.
   *
   * @param pose The new robot pose
   */
  public void resetOdometryAndEncoders(Pose2d pose) {
    // Reset the center encoders on either side
    m_centerLeftEncoder.setPosition(0);
    m_centerRightEncoder.setPosition(0);

    // Reset the odometry with the new pose
    m_odometry.resetPosition(pose, m_navx.getRotation2d());
  }

  /**
   * Updates the DifferentialDriveOdometry instance variable. It keeps track of where the robot is
   * on the field, but can get thrown off over time.
   */
  private void updateOdometry() {
    double currentLeftPosition = getLeftEncoderPosition();
    double currentRightPosition = getRightEncoderPosition();

    // Update the drive odometry
    m_odometry.update(m_navx.getRotation2d(), currentLeftPosition, currentRightPosition);
  }
}
