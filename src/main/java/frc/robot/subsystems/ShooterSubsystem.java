// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NavigableMap;
import java.util.TreeMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  // Flywheel motors
  private static CANSparkMax flywheelMotor;
  private static CANSparkMax flywheelMotor2;

  private SparkMaxPIDController m_pidController;

  private double targetPower = 0;

  private NetworkTable limelightTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv; 

  private NavigableMap<Double, Double> map = new TreeMap<>();
  
  /** Creates a new ArcShooter. */
  public ShooterSubsystem(ShooterConstants constants) {
    tyRPMMap();
    // Construct both flywheel motor objects
    flywheelMotor = new CANSparkMax(constants.LeftMotorCANId, MotorType.kBrushless);
    flywheelMotor2 = new CANSparkMax(constants.RightMotorCANId, MotorType.kBrushless);

    CANSparkMax maxes[] = {flywheelMotor, flywheelMotor2};

    for (CANSparkMax max : maxes) {
      // Restore to factory defaults only for this boot. Ensure we have a consistent
      // spark max setup even if someone changes values on the spark max flash
      max.restoreFactoryDefaults(false);

      // Set closed loop constants
      m_pidController = max.getPIDController();
      m_pidController.setP(constants.P);
      m_pidController.setI(constants.I);
      m_pidController.setD(constants.D);
      m_pidController.setFF(constants.kvVolts);

      limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
      ty = limelightTable.getEntry("ty");
    }

    // Set the pid controller to reference the first spark max
    m_pidController = flywheelMotor.getPIDController();



    // Invert the first motor and have the second motor follow also inverted
    flywheelMotor.setInverted(false);
    flywheelMotor2.follow(flywheelMotor, true);

    // Set the open loop ramp rate, really should be using closed loop but that is
    // currently not important
    //flywheelMotor.setOpenLoopRampRate(0.01);
  }

  @Override
  public void periodic() {
    targetPower = SmartDashboard.getNumber("Fly-Wheel-Target-Power", targetPower);
    flywheelMotor.set(targetPower);
    updateDashboard();

    double y = ty.getDouble(0.0);
    SmartDashboard.putNumber("LimelightY",y);
  }

  private void updateDashboard(){

    // The current flywheel speed
    SmartDashboard.putNumber("Fly-Wheel-RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Fly-Wheel-Target-Power", targetPower);
  }

  /**
   * Get the current flywheel RPM
   * @return
   */
  public double getFlywheelRPM(){
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

  /**
   * Run the shooter motor given a manual power
   */
  public void runShooter(double motorPower) {
    
    if (motorPower > 0.1) {
      flywheelMotor.set(motorPower);
    } else {
      flywheelMotor.set(0);
    }
    //flywheelMotor.setVoltage(10);
    
    //flywheelMotor.setVoltage(SmartDashboard.getNumber("Shooter Voltage", 10));
    
  }

  public void toggleShooter(){
    if(isRunning()){
      runShooter(0);
    }

    else{
      runShooter(0.5);
    }
  }

  public double RPM;
  public double maxRPM = 5676;
  public double distance;
  public int angle = 24;
  public double slope24 = 4000/24;
  public double slope4 = 1400/4;
  private double maxHeight; 
  private int shooterAngle = 24;
  private double shooterHeight = 0.3048;
  private final double gravity = 9.8;
  private int flywheelRadius = 4;
  private double targetHeight = 2.7432;


  public double getRelativeRPM(double dist){
    if (dist > 12){
      RPM = slope24 * dist;
    }
    else if (dist > 24){
      dist = 24;
      RPM = slope24 * dist;
    }
    else{
      RPM = slope4 * dist;
    }
    return RPM/ maxRPM;
  }

  public void shootFlywheels(double dst){
    runShooter(getRelativeRPM(dst));
  }


  public double RPMtoAngularVelocity(int rpm){
      double omega = (2 * Math.PI * rpm) / 60;
      return omega;
  }
  public double angularToLinearVelocity(double angularVelocity){
      //two flywheels 
      double velocity = RPMtoAngularVelocity(4000) * flywheelRadius; 
      return velocity; 
  }

  public double linearToRPM(double v){
      double angular = v / flywheelRadius;
      double rpm = (angular * 60) / (2 * Math.PI);
      return rpm;
  }

  public double ballMaxHeight(double dist, double initalVelocity, int launchAngle, double launchHeight){
      double maxHeight = ((dist * Math.tan(launchAngle)) + (0.5 * gravity * Math.pow(Math.cos(launchAngle),2)) + launchHeight);
      return maxHeight;
  }

  public double RPMforDistanceX(double dist, double launchAngle){
      //2 flywheels, rpm / 2 
      //dist is limelighttargeting.getdistance()
      double neededVelocity = Math.sqrt((dist * 9.8) / (Math.sin(2 * launchAngle)));
      double RPM = linearToRPM(neededVelocity);
      return RPM;
  }    

  public double ExtrapolateSlope(double ty1, double rpm1, double ty2, double rpm2){
    //make linaer model between rpm1 and rpm2
    double slope1 = rpm1/ ty1;
    double slope2 = rpm2/ ty2;
    double extraplolate = (slope1 + slope2)/2;
    /*
    double deltaRPM = rpm2 - rpm1;
    double deltaTy = ty2 - ty1;
    double slope = deltaRPM / deltaTy;*/
    return extraplolate;
  }

  public double rpmFromSlope(double ty, double slope){
    double desiredRPM = slope * ty; 
    return desiredRPM;
  }

  public void tyRPMMap(){
    map.put(-60.0,0.3);
    map.put(1.0,1.0);
    map.put(2.0,3.0);
    map.put(3.0, 4.0);
    map.put(42.0,3.0);
    map.put(15.0, 4.0);
    map.put(16.0,1.0);
    map.put(21.0,3.0);
    map.put(36.0, 4.0);
    map.put(24.0,3.0);
    map.put(32.0, 4.0);
    map.put(17.0,1.0);
    map.put(22.0,3.0);
    map.put(13.0, 4.0);
    map.put(12.0,3.0);
    map.put(60.0, 4.0); 
  }
  
  public double returnRPM(double ty){
    double above;
    double below; 
    try{
      above = map.ceilingKey(ty);
    }
    catch(Exception e){
      above = -1;
    }

    try{
      below = map.floorKey(ty);
    }
    catch(Exception e){
      below = -1;
    }

    double slope = ExtrapolateSlope(above, map.get(above), below, map.get(below));
    double rpm = slope * ty;
    return rpm;
  }

  public void test(){
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry ty = limelightTable.getEntry("ty"); // Y degrees
    double y = ty.getDouble(0.0);
    double rpm = returnRPM(y);
    SmartDashboard.putNumber("rpmVal",rpm);

  }

}