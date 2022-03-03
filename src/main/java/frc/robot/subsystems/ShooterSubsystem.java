// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  // Flywheel motors
  private static CANSparkMax flywheelMotor;
  private static CANSparkMax flywheelMotor2;

  private SparkMaxPIDController m_pidController;

  private double targetPower = 0;

  /** Creates a new ArcShooter. */
  public ShooterSubsystem(int leftMotor, int rightMotor) {

    // Construct both flywheel motor objects
    flywheelMotor = new CANSparkMax(leftMotor, MotorType.kBrushless);
    flywheelMotor2 = new CANSparkMax(rightMotor, MotorType.kBrushless);

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

    }

    // Set the pid controller to reference the first spark max
    m_pidController = flywheelMotor.getPIDController();



    // Invert the first motor and have the second motor follow also inverted
    flywheelMotor.setInverted(true);
    flywheelMotor2.follow(flywheelMotor, false);

    // Set the open loop ramp rate, really should be using closed loop but that is
    // currently not important
    //flywheelMotor.setOpenLoopRampRate(0.01);
  }

  @Override
  public void periodic() {
    flywheelMotor.set(targetPower);
    updateDashboard();
  }

  private void updateDashboard(){

    // Whether or not the fly wheel has reached full speed
    SmartDashboard.putBoolean("Fly-Wheel-Speed-Status", isFullSpeed());

    // The current flywheel speed
    SmartDashboard.putNumber("Fly-Wheel-RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Fly-Wheel-Target-Power", targetPower);
  }

  /**
   * Check if the geared up motor is at max rpm or > than 9000 rpm
   * 
   * @return status of full motor speed
   */
  public boolean isFullSpeed() {
    return Math.abs(getFlywheelRPM()) > 9000;
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

/**
  * Run the shooter motor at target power
  */
  public void runShooter() {
    flywheelMotor.set(targetPower);
    

  }
  public void toggleShooter(){
    if(isRunning()){
      runShooter(0);
    }else{
      runShooter(0.5);
    }
  }
  public void incrementTargetPower(double v) {
    targetPower += v;
    if (targetPower < 0) {
        targetPower = 0;
    }
    
    else if (targetPower > 1) {
        targetPower = 1;
    }

    //flywheelMotor.set(targetPower);
    //flywheelMotor.setVoltage(10);
    //flywheelMotor.setVoltage(10);//);
    m_pidController.setReference(SmartDashboard.getNumber("Shooter RPM", 3500), CANSparkMax.ControlType.kVelocity);
    
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
    double deltaRPM = rpm2 - rpm1;
    double deltaTy = ty2 - ty1;
    double slope = deltaRPM / deltaTy;
    return slope; 
  }

  public double rpmFromSlope(double ty, double slope){
    double desiredRPM = slope * ty; 
    return desiredRPM;
  }

  
  Dictionary<Double, Double> dictionary = new Hashtable<>();
  public void Dict(String[] args){
    
    //dictionary.put(ty value, rpm value) 
    // do every 6 inches -> about every 3 degress 
    dictionary.put(1.0,1.0);
    dictionary.put(27.3,2000.0);
    dictionary.put(12.0, 2000.0);
    dictionary.put(12.5, 2200.0);
    dictionary.put(13.0, 25000.0);
  }
  public double returnRPM(double ty){
    double lowTy = 0.5 * Math.floor(Math.abs(ty/0.5));
    double highTy = 0.5 * Math.ceil(Math.abs(ty/0.5));
    double higherTy = highTy + 0.5;
    double rpm = 0;
    if (Math.abs(ty - lowTy) > Math.abs(ty - highTy)){
      //double slope = ExtrapolateSlope(lowTy, )
      double slope = ExtrapolateSlope(lowTy, dictionary.get(lowTy), highTy, dictionary.get(highTy));
      rpm = ty * slope;
      return rpm;
    }
    else if (Math.abs(ty - highTy) > Math.abs(ty - lowTy)){
      double slope = ExtrapolateSlope(highTy, dictionary.get(highTy), higherTy, dictionary.get(higherTy));
      rpm = ty * slope;
      return rpm;
    }
    else if (ty == highTy){
      rpm = dictionary.get(highTy);
      return rpm;
    }
    else{
      rpm = dictionary.get(lowTy);
      return rpm;
    }
  
  }

  public double test(double ty){
    System.out.print(returnRPM(ty));
    return 0;
    }
  }

