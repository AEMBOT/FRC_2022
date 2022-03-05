package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.StraightPID.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightGyro extends CommandBase {
  private final PIDController m_gyroPID = new PIDController(0.01, 0, 0);
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kSVolts, kVVoltMetersPerSecond);
  private final ProfiledPIDController m_profile =
      new ProfiledPIDController(0.1, kI, kD, new TrapezoidProfile.Constraints(1.5, 0.5));

  private double m_distance;
  private DriveSubsystem m_drive;

  public DriveStraightGyro(double distance, DriveSubsystem drive) {
    // Set the gyro PID setpoint to the desired offset angle
    m_gyroPID.setSetpoint(0);
    m_gyroPID.enableContinuousInput(-180, 180);

    // Set setpoint of the drive PID controller
    m_profile.setGoal(new TrapezoidProfile.State(distance, 0));

    m_distance = distance;
    m_drive = drive;

    // Add drive subsystem as a requirement
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    // Ensure motors are in brake mode
    m_drive.setBrakeMode();

    // Reset the drive motor encoders
    m_drive.resetEncoders();

    // Reset the gyro
    m_drive.resetHeading();

    m_profile.reset(m_drive.getLeftEncoderPosition());
  }

  @Override
  public void execute() {
    // Use a PID controller to adjust the velocities to ensure the robot drives at the right angle
    double forwardSpeed =
        m_feedforward.calculate(m_profile.getSetpoint().velocity)
            + m_profile.calculate(m_drive.getLeftEncoderPosition(), m_distance);
    SmartDashboard.putNumber("Gyro Straight Speed", forwardSpeed);
    double angleAdjustment = m_gyroPID.calculate(m_drive.getHeading());
    m_drive.arcadeDrive(forwardSpeed, angleAdjustment, false);
  }

  @Override
  public boolean isFinished() {
    return m_profile.atGoal();
  }
}
