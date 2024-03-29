package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;

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

  // Linear system model of drivetrain
  private final LinearSystem<N2, N2, N2> m_drivetrainPlant =
      LinearSystemId.identifyDrivetrainSystem(kVLinear, kALinear, kVAngular, kAAngular);

  // Feedforward based on above linear system
  private final LinearPlantInversionFeedforward<N2, N2, N2> m_feedforward =
      new LinearPlantInversionFeedforward<>(m_drivetrainPlant, Constants.kRobotLoopPeriod);

  // This allows us to read angle information from the NavX
  //private final AHRS m_navx = new AHRS(SPI.Port.kMXP);

  // WPILib provides a convenience class for differential drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_centerLeftMotor, m_centerRightMotor);

  // This allows the robot to keep track of where it is on the field
  //private final DifferentialDriveOdometry m_odometry;

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

    // Motors should be in brake mode
    setBrakeMode();

    // Set current limits for all of the drive motors
    setCurrentLimits();

    // Set the SmartMotion constants for the center motors
    configureSparkMax(m_leftController);
    configureSparkMax(m_rightController);

    // Change encoder position readings to meters (per second)
    setupEncoderConversions();

    // Initialize the odometry to track where the robot is on the field
    //m_odometry = new DifferentialDriveOdometry(m_navx.getRotation2d(), 0, 0, new Pose2d());

    // Display the robot's position on a field widget on the dashboard
    SmartDashboard.putData(m_field);
  }

  @Override
  public void periodic() {
    // Keep track of where the robot is on the field
    Pose2d newPose = updateOdometry();

    // Update the field widget on the dashboard with the robot's current pose
    m_field.setRobotPose(newPose);
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
    controller.setP(kLinearP, slot);
    controller.setI(kLinearI, slot);
    controller.setD(kLinearD, slot);
    controller.setFF(kLinearFF, slot);

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
   * Drives the left and right wheels at the specified velocities using closed loop velocity control
   * on the Spark Maxes.
   *
   * @param leftVelocity The velocity of the left wheels, in meters per second.
   * @param rightVelocity The velocity of the right wheels, in meters per second.
   */
  public void tankDriveVelocities(double leftVelocity, double rightVelocity) {
    // Use a linear model of the drivetrain to calculate feedforward voltages
    // TODO: Handle cases where the feedforward voltage is over 12V
    Vector<N2> velocities = VecBuilder.fill(leftVelocity, rightVelocity);
    Matrix<N2, N1> feedforwards = m_feedforward.calculate(velocities);

    // TODO: Test if the static component makes other paths more accurate
    double leftVolts = feedforwards.get(0, 0) + Math.signum(leftVelocity) * kSLinear;
    double rightVolts = feedforwards.get(1, 0) + Math.signum(rightVelocity) * kSLinear;

    // Command the motors at the above feedforward voltages, using PID to correct for error
    m_leftController.setReference(leftVelocity, CANSparkMax.ControlType.kVelocity, 0, leftVolts);
    m_rightController.setReference(rightVelocity, CANSparkMax.ControlType.kVelocity, 0, rightVolts);

    // Feed the DifferentialDrive motor safety timer so it doesn't complain
    m_drive.feedWatchdog();
  }

  /**
   * Resets the previous velocities used for feedforward calculations in {@link
   * #tankDriveVelocities(double, double)}.
   */
  public void resetFeedforward() {
    m_feedforward.reset();
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
    return /*-m_navx.getYaw()*/ 0;
  }

  /**
   * Gets the angle the robot has rotated since the last gyro reset. Can be greater than 360
   * degrees.
   *
   * <p>Positive angles represent counterclockwise rotation, as most WPILib classes expect.
   */
  public double getAngle() {
    return /*-m_navx.getAngle()*/ 0;
  }

  /** Resets the robot heading to 0 degrees, as well as the drive motor encoders and odometry. */
  public void resetHeading() {
    //m_navx.zeroYaw();

    // Reset the odometry, reusing the previous pose on the field
    resetOdometryAndEncoders();
  }

  // ODOMETRY METHODS

  /** Returns the robot's current pose on the field in meters. */
  public Pose2d getPose() {
    return /*m_odometry.getPoseMeters()*/ new Pose2d();
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
    //m_odometry.resetPosition(m_navx.getRotation2d(), 0, 0, pose);
  }

  /**
   * Updates the DifferentialDriveOdometry instance variable. It keeps track of where the robot is
   * on the field, but can get thrown off over time.
   *
   * @return The new robot pose on the field.
   */
  private Pose2d updateOdometry() {
    double currentLeftPosition = getLeftEncoderPosition();
    double currentRightPosition = getRightEncoderPosition();

    // Update the drive odometry
    //return m_odometry.update(m_navx.getRotation2d(), currentLeftPosition, currentRightPosition);
    return new Pose2d();
  }

  // COMMAND FACTORIES

  /**
   * Creates a command that makes the robot drive the specified distance forward while following a
   * trapezoidal velocity profile.
   *
   * @param meters The distance for the robot to drive, in meters.
   * @return The command that makes the robot drive the specified distance.
   */
  public Command driveMetersCommand(double meters) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecondSquared),
            new TrapezoidProfile.State(meters, 0)),
        state -> tankDriveVelocities(state.velocity, state.velocity),
        this);
  }
}
