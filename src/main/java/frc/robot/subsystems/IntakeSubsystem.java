package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  // motor controller
  private final CANSparkMax liftRight = 
      new CANSparkMax(11, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_liftRightEncoder = liftRight.getEncoder();

  private SparkMaxPIDController m_rightcontroller = liftRight.getPIDController();

  // whether the subsystem is successfully homed to its max point
  public boolean homingComplete = false;

  // The encoder positions for the lowest and highest allowed
  private double m_lowestAllowedPosition;
  private double m_highestAllowedPosition;

  private boolean rollerRunning = false;

  // motor geared 125:1 -> 24:72 gearing
  public IntakeSubsystem() {

    // TODO: set position conversion factor
    double factor = 90/18.85;

    liftRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    liftRight.setSmartCurrentLimit(5,0,20000);

    liftRight.setInverted(true);

    // set the position conversion factors for the lift encoders
    m_liftRightEncoder.setPositionConversionFactor(factor);


    //m_rightcontroller.setReference(10.0, CANSparkMax.ControlType.kSmartMotion,0);
  }
/*
  private void setSmartMotionConstants(SparkMaxPIDController controller){
    controller.setFF(kFF,0);
    controller.setOutputRange(kMinOutpout, kMaxOutput, slot);
    controller.setSmartMotionMaxVelocty(kMaxVel, slot);
    controller.setSmartMotionMinOutputVelocity(kMinVel,slot);
    controller.setSmartMotionMaxAccel(kMaxAcc, slot);
  }*/

  public void lowerIntake() {
    m_rightcontroller.setReference(m_highestAllowedPosition, CANSparkMax.ControlType.kPosition);
  }

  public void raiseIntake() {
    m_rightcontroller.setReference(m_lowestAllowedPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setLiftPower(double power) {
    liftRight.set(power);
  }

  /** gets the encoder position of the left lift */
  public double getLiftPosition() {
    return liftRight.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift", liftRight.getOutputCurrent());
    SmartDashboard.putNumber("Lift Position", m_liftRightEncoder.getPosition());
  }
}
