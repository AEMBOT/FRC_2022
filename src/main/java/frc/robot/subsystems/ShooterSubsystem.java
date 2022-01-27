// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  // Flywheel motors
  private static CANSparkMax flywheelMotor;
  private CANSparkMax flywheelMotor2;

  private boolean shooterToggled = false;

  private double targetPower = 0;

  /** Creates a new ArcShooter. */
  public ShooterSubsystem(int leftMotor, int rightMotor) {

    // Construct both flywheel motor objects
    flywheelMotor = new CANSparkMax(leftMotor, MotorType.kBrushless);
    flywheelMotor2 = new CANSparkMax(rightMotor, MotorType.kBrushless);

    // Invert the first motor and have the second motor follow also inverted
    flywheelMotor.setInverted(true);
    flywheelMotor2.follow(flywheelMotor, false);

    // Set the open loop ramp rate, really should be using closed loop but that is
    // currently not important
    flywheelMotor.setOpenLoopRampRate(2.5);
  }

  @Override
  public void periodic() {
      distance = LimeLightTargeting.getDistance();
      shootFlywheels(distance);
      updateDashboard();
  }

  private void updateDashboard(){

    // Wether or not the fly wheel has reached full speed
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
    } else
      flywheelMotor.set(0);
  }

/**
  * Run the shooter motor at target power
  */
  public void runShooter() {
    flywheelMotor.set(targetPower);

  }

  public void incrementTargetPower(double v) {
    targetPower += v;
    if (targetPower < 0) {
        targetPower = 0;
    }
    
    else if (targetPower > 1) {
        targetPower = 1;
    }

    flywheelMotor.set(targetPower);
  }

  public double RPM;
  public double maxRPM = 5676;
  public double distance;
  public int angle = 24;
  public double slope24 = 4000/24;
  public double slope4 = 1400/4;

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
}