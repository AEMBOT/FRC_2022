package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class ClosedLoopSparkMax extends CANSparkMax {

  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;

  // Loop coefficients
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

  public ClosedLoopSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);

    m_pidController = getPIDController();
    m_encoder = getEncoder();

    // need to restore factory defaults
    restoreFactoryDefaults();

    // initialize PID coefficients
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // initialize SmartMotion values
    m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, 0);
    m_pidController.setSmartMotionMaxAccel(maxAcc, 0);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, 0);
  }

  /**
   * Uses the closed loop control onboard the SparkMax to move some number of rotations
   *
   * @return error as an REVLibError, REVLibError.kOk == successful
   */
  public REVLibError runToSetPoint(double setPoint) {
    return m_pidController.setReference(setPoint, ControlType.kPosition);
  }

  /**
   * Uses the "Smart Motion" closed loop control onboard the SparkMax to move some number of
   * rotations
   *
   * @return error as an REVLibError, REVLibError.kOk == successful
   */
  public REVLibError runToSetPointSmartMotion(double setPoint) {
    return m_pidController.setReference(setPoint, ControlType.kSmartMotion);
  }

  /**
   * Uses the closed loop control onboard the SparkMax to run at a target velocity
   *
   * @return error as an REVLibError, REVLibError.kOk == successful
   */
  public REVLibError setVelocity(double setPoint) {
    return m_pidController.setReference(setPoint, ControlType.kVelocity);
  }

  /*  Coefficient Mutators:
   *
   *   These set the coefficients for the SparkMax's PID loop, and return `this` so they can be chained, example:
   *       double p, i, d;
   *       ClosedLoopSparkMax motor = new ClosedLoopSparkMax(id, MotorType.kBrushless).kP(p).kI(i).kD(d);
   *
   *   Each method checks if the value needs to be changed before applying it
   *   because the underlying set methods should be called infrequently.
   */

  public ClosedLoopSparkMax kP(double p) {
    if (p != kP) {
      m_pidController.setP(p);
      kP = p;
    }
    return this;
  }

  public ClosedLoopSparkMax kI(double i) {
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    return this;
  }

  public ClosedLoopSparkMax kD(double d) {
    if (d != kD) {
      m_pidController.setD(d);
      kD = d;
    }
    return this;
  }

  public ClosedLoopSparkMax kIz(double iz) {
    if (iz != kIz) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    return this;
  }

  public ClosedLoopSparkMax kFF(double ff) {
    if (ff != kFF) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    return this;
  }

  public ClosedLoopSparkMax setOutputRange(double min, double max) {
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);

      kMinOutput = min;
      kMaxOutput = max;
    }

    return this;
  }

  public ClosedLoopSparkMax setSmartMotionMaxVelocity(double maxV) {
    if (maxV != maxVel) {
      m_pidController.setSmartMotionMaxVelocity(maxV, 0);
      maxVel = maxV;
    }

    return this;
  }

  public ClosedLoopSparkMax setSmartMotionMinOutputVelocity(double minV) {
    if (minV != minVel) {
      m_pidController.setSmartMotionMinOutputVelocity(minV, 0);
      minVel = minV;
    }
    return this;
  }

  public ClosedLoopSparkMax setSmartMotionMaxAccel(double maxA) {
    if (maxA != maxAcc) {
      m_pidController.setSmartMotionMaxAccel(maxA, 0);
      maxAcc = maxA;
    }
    return this;
  }

  public ClosedLoopSparkMax setSmartMotionAllowedClosedLoopError(double allE) {
    if (allE != allowedErr) {
      m_pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
      allowedErr = allE;
    }
    return this;
  }
}
