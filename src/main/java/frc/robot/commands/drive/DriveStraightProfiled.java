package frc.robot.commands.drive;

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
            drive.arcadeDrive(-(output + m_feedforward.calculate(setpoint.velocity)), 0, false),
        drive);

    getController().setTolerance(kDriveToleranceMeters, kDriveVelocityToleranceMetersPerSecond);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Setpoint Velocity", getController().getSetpoint().velocity);
    SmartDashboard.putNumber(
        "Actual Velocity",
        getController().getSetpoint().velocity + getController().getVelocityError());
    SmartDashboard.putNumber("Measurement", m_measurement.getAsDouble());
    SmartDashboard.putBoolean("At goal", getController().atGoal());
    SmartDashboard.putNumber("Goal: ", getController().getGoal().position);

    super.execute();
  }

  @Override
  public boolean isFinished() {
    return super.getController().atGoal();
  }
}
