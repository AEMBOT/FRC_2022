package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.DrivetrainConstants.StraightPID.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

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
      new LinearPlantInversionFeedforward<>(m_drivetrainPlant, 0.02);

  // This allows us to read angle information from the NavX
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP);

  // WPILib provides a convenience class for differential drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_centerLeftMotor, m_centerRightMotor);

  // This allows the robot to keep track of where it is on the field
  private final DifferentialDriveOdometry m_odometry;

  // Used to display the current robot position
  private final Field2d m_field = new Field2d();

  private DifferentialDrivetrainSim m_driveSim;
  private SimDouble m_yawAngleSim;

  /** Creates a new DriveSubsystem. */
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

    if (RobotBase.isSimulation()) {
      // Simulate the center spark maxes on either side
      var revPhysicsSim = REVPhysicsSim.getInstance();
      revPhysicsSim.addSparkMax(m_centerLeftMotor, DCMotor.getNEO(3));
      revPhysicsSim.addSparkMax(m_centerRightMotor, DCMotor.getNEO(3));

      // Initialize the drivetrain simulation using SysId values & other drivetrain properties
      m_driveSim =
          new DifferentialDrivetrainSim(
              m_drivetrainPlant,
              DCMotor.getNEO(3),
              7.56,
              kEffectiveTrackWidth,
              Units.inchesToMeters(3),
              // These are a bunch of standard deviations that should probably be tuned
              VecBuilder.fill(0.005, 0.005, 0.0001, 0.05, 0.05, 0.005, 0.005));

      // Allows for the updating of the simulated robot heading read by the NavX
      var navxHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      m_yawAngleSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navxHandle, "Yaw"));
    }
  }

  @Override
  public void periodic() {
    // Keep track of where the robot is on the field
    Pose2d newPose = updateOdometry();

    // Update the field widget on the dashboard with the robot's current pose
    m_field.setRobotPose(newPose);
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation with input voltages supplied via tankDriveVelocities
    m_driveSim.update(0.020);

    // Update the simulated center motors
    REVPhysicsSim.getInstance().run();

    // Encoders have to be updated manually because they move super slowly otherwise
    m_centerLeftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    m_centerRightEncoder.setPosition(m_driveSim.getRightPositionMeters());

    // NavX angle is clockwise-positive, which is the opposite of the sim heading
    m_yawAngleSim.set(-m_driveSim.getHeading().getDegrees());
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
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(speed, rotation, squareInputs);

    // For now, wheel speeds are just scaled by the max (autonomous) velocity
    tankDriveVelocities(
        wheelSpeeds.left * kMaxVelocityMetersPerSecond,
        wheelSpeeds.right * kMaxVelocityMetersPerSecond);
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
    // TODO: StateSpaceUtil.desaturateInputVector could be used to make sure the voltages don't go
    // over 12V, but the velocities would also have to be scaled
    Vector<N2> velocities = VecBuilder.fill(leftVelocity, rightVelocity);
    Matrix<N2, N1> feedforwards = m_feedforward.calculate(velocities);

    double leftVolts = feedforwards.get(0, 0);
    double rightVolts = feedforwards.get(1, 0);

    // Command the motors at the above feedforward voltages, using PID to correct for error
    m_leftController.setReference(leftVelocity, CANSparkMax.ControlType.kVelocity, 0, leftVolts);
    m_rightController.setReference(rightVelocity, CANSparkMax.ControlType.kVelocity, 0, rightVolts);

    if (RobotBase.isSimulation()) {
      // Log feedforward voltages to the dashboard during simulation
      SmartDashboard.putNumber("Left feedforward", leftVolts);
      SmartDashboard.putNumber("Right feedforward", rightVolts);

      // Update the drivetrain simulation when simulating
      m_driveSim.setInputs(leftVolts, rightVolts);
    }

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
   *
   * @return The new robot pose on the field.
   */
  private Pose2d updateOdometry() {
    double currentLeftPosition = getLeftEncoderPosition();
    double currentRightPosition = getRightEncoderPosition();

    // Update the drive odometry
    return m_odometry.update(m_navx.getRotation2d(), currentLeftPosition, currentRightPosition);
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
