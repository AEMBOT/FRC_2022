// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.NavigableMap;
import java.util.TreeMap;

public class ShooterSubsystem extends SubsystemBase {

  // Flywheel motors
  private static CANSparkMax flywheelMotor;
  private static CANSparkMax flywheelMotor2;

  private RelativeEncoder flywheelEncoder;
  private RelativeEncoder flywheel2Encoder;

  private SparkMaxPIDController m_pidController;

  private DriveSubsystem m_driveSubsystem;
  
  private double targetPower = 0;

  private NetworkTable limelightTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;

  private NavigableMap<Double, Double> map = new TreeMap<>();

  /** Creates a new ArcShooter. */
  public ShooterSubsystem() {
    tyRPMMap();
    // Construct both flywheel motor objects
    flywheelMotor =
        new CANSparkMax(Constants.ShooterConstants.LeftMotorCANId, MotorType.kBrushless);
    flywheelMotor2 =
        new CANSparkMax(Constants.ShooterConstants.RightMotorCANId, MotorType.kBrushless);

    CANSparkMax maxes[] = {flywheelMotor, flywheelMotor2};

    for (CANSparkMax max : maxes) {
      // Restore to factory defaults only for this boot. Ensure we have a consistent
      // spark max setup even if someone changes values on the spark max flash
      max.restoreFactoryDefaults(false);

      // Set closed loop constants
      m_pidController = max.getPIDController();
      m_pidController.setP(ShooterConstants.P);
      m_pidController.setI(ShooterConstants.I);
      m_pidController.setD(ShooterConstants.D);
      m_pidController.setFF(ShooterConstants.kvVolts);
      m_pidController.setOutputRange(ShooterConstants.minOutput, ShooterConstants.maxOutput);
      int smartMotionSlot = 0;
      m_pidController.setSmartMotionMaxVelocity(ShooterConstants.maxVel,smartMotionSlot);
      m_pidController.setSmartMotionMinOutputVelocity(ShooterConstants.minVel, smartMotionSlot);
      m_pidController.setSmartMotionMaxAccel(ShooterConstants.maxAcc, smartMotionSlot);
      //m_pidController.setSmartMotionAllowedClosedLoopError(ShooterConstants.allowedError, smartMotionSlot);

      limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
      ty = limelightTable.getEntry("ty");
    }

    // Set the pid controller to reference the first spark max
    m_pidController = flywheelMotor.getPIDController();

    // Invert the first motor and have the second motor follow also inverted
    flywheelMotor.setInverted(false);
    flywheelMotor2.follow(flywheelMotor, true);

    flywheelEncoder = flywheelMotor.getEncoder();
    flywheel2Encoder = flywheelMotor2.getEncoder();

    // Set the open loop ramp rate, really should be using closed loop but that is
    // currently not important
    // flywheelMotor.setOpenLoopRampRate(0.01);
  }

  @Override
  public void periodic() {
    targetPower = SmartDashboard.getNumber("Fly-Wheel-Target-Power", targetPower);
    flywheelMotor.set(targetPower);
    updateDashboard();

    double y = ty.getDouble(0.0);

    SmartDashboard.putNumber("LimelightY", y);

    double fwhlM1ActualRpm = flywheelEncoder.getVelocity();
    double fwhlM2ActualRpm = flywheel2Encoder.getVelocity();
    SmartDashboard.putNumber("Flywheel1 RPM", fwhlM1ActualRpm);
    SmartDashboard.putNumber("Flywheel2 RPM", fwhlM2ActualRpm);
  }

  private void updateDashboard() {

    // The current flywheel speed
    SmartDashboard.putNumber("Fly-Wheel-RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Fly-Wheel-Target-Power", targetPower);
  }

  /**
   * Get the current flywheel RPM
   *
   * @return
   */
  public double getFlywheelRPM() {
    return flywheelMotor.getEncoder().getVelocity();
  }

  /**
   * Determines whether or not the shooter is running
   *
   * @return shooter status
   */
  public boolean isRunning() {
    if (Math.abs(flywheelMotor.get()) > 0.05) {
      return true;
    }
    return false;
  }

  /** Run the shooter motor given a manual power */
  public void runShooter(double motorPower) {

    if (motorPower > 0.1) {
      flywheelMotor.set(motorPower);
    } else {
      flywheelMotor.set(0);
    }
    // flywheelMotor.setVoltage(10);

    // flywheelMotor.setVoltage(SmartDashboard.getNumber("Shooter Voltage", 10));

  }

  public void toggleShooter() {
    if (isRunning()) {
      runShooter(0);
    } else {
      runShooter(0.5);
    }
  }

  public double RPM;
  public double maxRPM = 5676;
  public double distance;
  public int angle = 24;
  public double slope24 = 4000 / 24;
  public double slope4 = 1400 / 4;
  private int shooterAngle = 24;
  private double shooterHeight = 0.3048;
  private final double gravity = 9.8;
  private int flywheelRadius = 4;
  private double targetHeight = 2.7432;


  public double RPMtoAngularVelocity(int rpm) {
    double omega = (2 * Math.PI * rpm) / 60;
    return omega;
  }

  public double angularToLinearVelocity(double angularVelocity) {
    // two flywheels
    double velocity = RPMtoAngularVelocity(4000) * flywheelRadius;
    return velocity;
  }

  public double linearToRPM(double v) {
    double angular = v / flywheelRadius;
    double rpm = (angular * 60) / (2 * Math.PI);
    return rpm;
  }

  public double LinearInterpolation(double ty1, double rpm1, double ty2, double rpm2, double findty) {
    double slope = (rpm2 - rpm1 ) /(ty2 - ty1);
    double rpm = rpm1 + slope * (findty - ty1);

    return rpm;
  }

  public void tyRPMMap() {
    map.put(-0.266,2650.0);
    map.put(-5.10,2885.0);
    map.put(-8.55,3100.0);
    map.put(-11.15,3325.0);
  }

  public double returnRPM(double ty) {
    double above;
    double below;
    try {
      above = map.ceilingKey(ty);
    } catch (Exception e) {
      above = -1;
    }

    try {
      below = map.floorKey(ty);
    } catch (Exception e) {
      below = -1;
    }

    double rpm = LinearInterpolation(above, map.get(above), below, map.get(below),ty);
    return rpm;
  }

  public void runAtMapRPM() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = limelightTable.getEntry("ty"); // Y degrees
    double yAngle = ty.getDouble(0.0);
    double rpm;
    try {
      rpm = returnRPM(yAngle);
    } catch (NullPointerException e) {
      rpm = 0;
    }
    m_pidController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  public void stopShooter() {
    flywheelMotor.set(0);
  }

  public void test() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    

    NetworkTableEntry ty = limelightTable.getEntry("ty"); // Y degrees
    double y = ty.getDouble(0.0);
    double rpm = returnRPM(y);
    SmartDashboard.putNumber("rpmVal", rpm);
    m_pidController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    //flywheelMotor.set(rpm);
    //flywheelMotor2.set(rpm);

    NetworkTableEntry tx = limelightTable.getEntry("tx");
    double angle = tx.getDouble(0.0);
    SmartDashboard.putNumber("xFixDegrees", angle);
    
    //TurnToAngleProfiled turn = new TurnToAngleProfiled(angle,m_driveSubsystem);
    // turn.TurnToAngleProfiled(angle,m_driveSubsystem); 
    
    //these are now being outputted periodically in the smart dashboard 
    //double fwhlM1ActualRpm = flywheelEncoder.getVelocity();
    //double fwhlM2ActualRpm = flywheel2Encoder.getVelocity();
  }
}
