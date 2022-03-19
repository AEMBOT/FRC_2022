package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  // motor controller
  private final CANSparkMax lift = 
      new CANSparkMax(Constants.IntakeConstants.LiftCANId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_liftEncoder = lift.getEncoder();

  private SparkMaxPIDController m_rightcontroller = lift.getPIDController();

  // whether the subsystem is successfully homed to its max point
  private boolean homingComplete = false;
  private double homePosition = 0; // Let's define home as the hard limit at the highest position

  // The encoder positions for the lowest and highest allowed
  private double m_lowestAllowedPosition;
  private double m_highestAllowedPosition;

  private boolean rollerRunning = false;

  // motor geared 125:1 -> 24:72 gearing
  public IntakeSubsystem() {

    // TODO: set position conversion factor
    double factor = 90/18.85;

    lift.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lift.setSmartCurrentLimit(Constants.IntakeConstants.kMaxLiftCurrent,0,20000);

    lift.setInverted(true);

    // set the position conversion factors for the lift encoders
    m_liftEncoder.setPositionConversionFactor(factor);


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

  public boolean isHomingComplete() {
    return homingComplete;
  }
  public void setHomePosition() {
    lift.getEncoder().setPosition(0);
    homingComplete = true;
  }

  public boolean isAtHardLimit() {
    return (lift.getOutputCurrent() > Constants.IntakeConstants.kMaxLiftCurrent);
  }

  public void lowerIntake() {
    m_rightcontroller.setReference(m_highestAllowedPosition, CANSparkMax.ControlType.kPosition);
  }

  public void raiseIntake() {
    m_rightcontroller.setReference(m_lowestAllowedPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setLiftPower(double power) {
    lift.set(power);
  }

  /** gets the encoder position of the left lift */
  public double getLiftPosition() {
    return lift.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift Position", m_liftRightEncoder.getPosition());
    SmartDashboard.putNumber("Lift", lift.getOutputCurrent());
  }
}
