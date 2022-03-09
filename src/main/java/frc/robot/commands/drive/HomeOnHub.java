package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.TurnPID.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightTargeting;

public class HomeOnHub extends CommandBase {
  private LimeLightTargeting m_limelight;
  private DriveSubsystem m_drive;

  // TODO: If we end up using both PID and a motion profile their PID constants will have to be
  // tuned separately
  private final ProfiledPIDController m_turnProfile =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              kMaxVelocityDegreesPerSecond, kMaxAccelerationDegreesPerSecondSquared));
  private final PIDController m_turnPID = new PIDController(kP, kI, kD);
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kSVolts, kVVoltDegreesPerSecond);

  private boolean m_profileFinished = false;

  public HomeOnHub(LimeLightTargeting limelight, DriveSubsystem drive) {
    m_limelight = limelight;
    m_drive = drive;
  }

  @Override
  public void initialize() {
    // We want the hub to be directly in front of the robot
    m_turnProfile.setGoal(0);
    m_turnPID.setSetpoint(0);
  }

  @Override
  public void execute() {
    double hubAngle = m_limelight.getX();
    double turnPower;

    //Snap to target if it has been found
    if (m_limelight.hasValidTarget()){
      // Use the profile if it hasn't finished
      if (!m_profileFinished) {
        double velocitySetpoint = m_turnProfile.getSetpoint().velocity;
        double output = m_turnProfile.calculate(hubAngle);
        turnPower = velocitySetpoint + output;

        m_profileFinished = m_turnProfile.atSetpoint();
      }

      // Correct any error accumulated in profiled motion with a pure PID controller
      else {
        double output = m_turnPID.calculate(hubAngle);
        turnPower = m_feedforward.calculate(output);
      }

      m_drive.arcadeDrive(0, turnPower, false);
    } else {
      //If a target is not found, spin until one is 
      m_drive.arcadeDrive(0, 0.15, false);
    }

    
  }

  @Override
  public void end(boolean _interrupted) {
    m_drive.stopMotors();
  }

  @Override
  public boolean isFinished() {
    // Just use profile for now to see how accurate it is
    return m_profileFinished;
    // return m_turnPID.atSetpoint();
  }
}
