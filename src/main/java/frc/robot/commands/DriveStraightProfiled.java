package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.StraightPID.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightProfiled extends ProfiledPIDCommand {
  private static SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kSVolts, kVVoltMetersPerSecond);
  private DriveSubsystem m_drive;

  public DriveStraightProfiled(double distance, DriveSubsystem drive) {
    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                kMaxVelocityMetersPerSecond, kMaxAccelerationMeterPerSecondSquared)),
        drive::getLeftEncoderPosition,
        distance,
        (output, setpoint) ->
            drive.arcadeDrive(output + m_feedforward.calculate(setpoint.velocity), 0, false),
        drive);

    m_controller.setTolerance(kDriveToleranceMeters, kDriveVelocityMetersPerSecond);
    m_drive = drive;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Setpoint Velocity", m_controller.getSetpoint().velocity);
    SmartDashboard.putNumber("Actual Velocity", m_drive.getLeftEncoderVelocity());
  }

  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}
