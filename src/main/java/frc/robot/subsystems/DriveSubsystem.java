package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.StraightPID.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Drive motors
  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(kLeftFront, MotorType.kBrushless);
  private final CANSparkMax m_centerLeftMotor = new CANSparkMax(kLeftCenter, MotorType.kBrushless);
  private final CANSparkMax m_backLeftMotor = new CANSparkMax(kLeftBack, MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(kRightFront, MotorType.kBrushless);
  private final CANSparkMax m_centerRightMotor =
      new CANSparkMax(kRightCenter, MotorType.kBrushless);
  private final CANSparkMax m_backRightMotor = new CANSparkMax(kRightBack, MotorType.kBrushless);

  // Encoders for center motors
  private final RelativeEncoder m_centerLeftEncoder = m_centerLeftMotor.getEncoder();
  private final RelativeEncoder m_centerRightEncoder = m_centerRightMotor.getEncoder();

  // Internal PID controllers for center motors
  private final SparkMaxPIDController m_leftController = m_centerLeftMotor.getPIDController();
  private final SparkMaxPIDController m_rightController = m_centerRightMotor.getPIDController();

  // This allows us to read angle information from the NavX
  private final AHRS m_ahrs = new AHRS(SPI.Port.kMXP);

  // WPILib provides a convenience class for differential drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_centerLeftMotor, m_centerRightMotor);

  // TODO: Figure out if we actually need this
  // This allows the robot to keep track of where it is on the field
  private DifferentialDriveOdometry m_odometry;
  private Pose2d m_lastResetPose;
  private double m_leftPosition = 0;
  private double m_rightPosition = 0;

  // Whether or not to log debug information to the SmartDashboard
  private final boolean m_debug = true;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Restore the default settings for all of the motors
    restoreMotorFactoryDefaults();

    // Invert the left motors to drive in the correct direction
    // m_leftMotors.setInverted(true);
    m_centerLeftMotor.setInverted(true);

    // Have the front/back motors mirror power outputs from the center motors
    m_frontLeftMotor.follow(m_centerLeftMotor);
    m_backLeftMotor.follow(m_centerLeftMotor);

    m_frontRightMotor.follow(m_centerRightMotor);
    m_backRightMotor.follow(m_centerRightMotor);

    // TODO: Test native Spark MAX voltage compensation
    // Enable voltage compensation for all of the motors
    // m_frontLeftMotor.enableVoltageCompensation(nominalVoltage);
    // m_centerLeftMotor.enableVoltageCompensation(nominalVoltage);
    // m_backLeftMotor.enableVoltageCompensation(nominalVoltage);
    // m_frontRightMotor.enableVoltageCompensation(nominalVoltage);
    // m_centerRightMotor.enableVoltageCompensation(nominalVoltage);
    // m_backRightMotor.enableVoltageCompensation(nominalVoltage);

    // Set the SmartMotion constants for the center motors
    setSmartMotionConstants(m_leftController);
    setSmartMotionConstants(m_rightController);

    // Reset the encoders & change their position readings to meters
    resetEncoders();
    setupEncoderConversions();

    // Initialize the tracking of the robot's position on the field
    // m_odometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()));

    // Keep track of the place where the odometry was last reset
    // m_lastResetPose = m_odometry.getPoseMeters();
  }

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

  /** Sets various Smart Motion constants for a Spark Max PID controller */
  private void setSmartMotionConstants(SparkMaxPIDController controller) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setIZone(kIz);

    controller.setOutputRange(kMinOutput, kMaxOutput);

    controller.setFF(kFF);

    int slot = 0;
    controller.setSmartMotionMaxVelocity(kMaxVel, slot);
    controller.setSmartMotionMinOutputVelocity(kMinVel, slot);
    controller.setSmartMotionMaxAccel(kMaxAcc, slot);
    controller.setSmartMotionAllowedClosedLoopError(kAllowedErr, slot);
  }

  @Override
  public void periodic() {
    // Keep track of where the robot is on the field
    // updateOdometry();

    // Log drive-related informatin to SmartDashboard if specified
    if (m_debug) {
      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putNumber("Rotation Velocity", getRotationRate());

      // Motor positions (meters)
      SmartDashboard.putNumber("Left Position", getLeftEncoderPosition());
      SmartDashboard.putNumber("Right Position", getRightEncoderPosition());

      // Motor velocities (meters per second)
      SmartDashboard.putNumber("Left Velocity", m_centerLeftEncoder.getVelocity());
      SmartDashboard.putNumber("Right Velocity", m_centerRightEncoder.getVelocity());

      // Motor powers (-1 to 1)
      SmartDashboard.putNumber("Left Power", m_centerLeftMotor.get());
      SmartDashboard.putNumber("Right Power", m_centerRightMotor.get());
    }
  }

  // BASIC DRIVE METHODS

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
    SmartDashboard.putNumber("Arcade forward", speed);
    /*

    // Reimplement WPILib arcade drive so that we get setVoltage() functionality
    speed = MathUtil.applyDeadband(speed, DifferentialDrive.kDefaultDeadband);
    rotation = MathUtil.applyDeadband(rotation, DifferentialDrive.kDefaultDeadband);

    var speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, squareInputs);

    m_leftMotors.setVoltage(speeds.left * DifferentialDrive.kDefaultMaxOutput * nominalBatteryVoltage);
    m_rightMotors.setVoltage(speeds.right * DifferentialDrive.kDefaultMaxOutput * nominalBatteryVoltage);

    SmartDashboard.putNumber("Output Voltage (r)", speeds.right* DifferentialDrive.kDefaultMaxOutput * nominalBatteryVoltage);
    SmartDashboard.putNumber("Output Voltage (l)", speeds.left * DifferentialDrive.kDefaultMaxOutput * nominalBatteryVoltage);
    */
  }

  private void restoreMotorFactoryDefaults() {
    m_backLeftMotor.restoreFactoryDefaults();
    m_centerLeftMotor.restoreFactoryDefaults();
    m_frontLeftMotor.restoreFactoryDefaults();

    m_backRightMotor.restoreFactoryDefaults();
    m_centerRightMotor.restoreFactoryDefaults();
    m_frontRightMotor.restoreFactoryDefaults();
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

  /** Resets the center drive motor encoders to a position of 0 */
  public void resetEncoders() {
    m_centerLeftEncoder.setPosition(0);
    m_centerRightEncoder.setPosition(0);

    // TODO: If odometry is added back in, previous positions also have to be
    // updated
  }

  /**
   * Changes encoder readings to meters and meters per second for position and velocity,
   * respectively.
   */
  private void setupEncoderConversions() {
    // TODO: Maybe compensate for lower right encoder readings?
    // Convert revolutions to meters
    m_centerLeftEncoder.setPositionConversionFactor(kMetersPerMotorRotation);
    m_centerRightEncoder.setPositionConversionFactor(kMetersPerMotorRotation);

    // Convert RPM to meters per second
    m_centerLeftEncoder.setVelocityConversionFactor(kRPMToMetersPerSecond);
    m_centerRightEncoder.setVelocityConversionFactor(kRPMToMetersPerSecond);
  }

  // SMARTMOTION METHODS

  /** Use the internal Spark Max velocity control */
  public void driveAtVelocity(double left, double right) {
    m_drive.feed();
    m_leftController.setReference(left, ControlType.kVelocity);
    m_rightController.setReference(right, ControlType.kVelocity);
  }

  /**
   * Runs the motors on both sides of the robot using SmartMotion.
   *
   * @param left The distance to move the left motor
   * @param right The distance to move the right motor
   */
  public void smartMotionToPosition(double left, double right) {
    m_drive.feed();
    m_rightController.setReference(right, ControlType.kSmartMotion);
    m_leftController.setReference(left, ControlType.kSmartMotion);
  }

  /**
   * Runs the motors on both sides of the robot using SmartMotion.
   *
   * @param distance The distance to drive (in meters)
   */
  public void smartMotionToPosition(double distance) {
    smartMotionToPosition(distance, distance);
  }

  /**
   * Returns true if the motors are close enough to a given goal position.
   *
   * @param goal The goal position of the motors
   */
  public boolean smartMotionAtGoal(double goal) {
    double leftPosition = getLeftEncoderPosition();
    double rightPosition = getRightEncoderPosition();
    return inRangeInclusive(goal, kAllowedErr, leftPosition)
        || inRangeInclusive(goal, kAllowedErr, rightPosition);
  }

  /** Checks if a value is within a given margin of a goal value. */
  private boolean inRangeInclusive(double goal, double margin, double val) {
    return goal - margin <= val && val <= goal + margin;
  }

  // NAVX METHODS

  /** Gets the heading of the robot. Ranges from -180 to 180 degrees. */
  public double getHeading() {
    // Negated to mirror signs on a coordinate plane (+ -> counterclockwise and vice
    // versa)
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
    // getRate only returns the difference in angles, so it has to be multiplied by
    // the NavX update
    // frequency
    // (see https://github.com/kauailabs/navxmxp/issues/69)
    return -m_ahrs.getRate() * m_ahrs.getActualUpdateRate();
  }

  /** Resets the robot heading to 0 degrees, as well as the odometry. */
  public void resetHeading() {
    // TODO: This fails if the NavX is calibrating, so it might be a good idea to
    // check for that
    m_ahrs.zeroYaw();

    // Resetting the odometry also requires zeroing the encoders
    // TODO: Either mention this in the method documentation or call it separately
    resetEncoders();

    /*
     * TODO: Determine if this is appropriate for this year
     * // Reuse the previous pose on the field, compensating for the change in
     * heading
     * Pose2d prev_pose = m_odometry.getPoseMeters();
     * m_odometry.resetPosition(prev_pose, new
     * Rotation2d(Units.degreesToRadians(getHeading())));
     */
  }

  // ODOMETRY METHODS (might not be necessary)

  /** Gets the displacent of the robot relative to the last time the odometry was reset */
  public double getResetDisplacement() {
    return -1;
    /*
    // TODO: Convert to use feet
    Pose2d currentOffset = m_odometry.getPoseMeters().relativeTo(m_lastResetPose);
    return currentOffset.getTranslation().getDistance(new Translation2d());
    */
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
