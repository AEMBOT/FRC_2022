package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.StraightPID.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants.StraightPID;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightProfiled extends ProfiledPIDCommand {

  public DriveStraightProfiled(double distance, DriveSubsystem drive, StraightPID constants) {
    super(
        new ProfiledPIDController(
            constants.kP,
            constants.kI,
            constants.kD,
            new TrapezoidProfile.Constraints(
                constants.kMaxVelocityMetersPerSecond, constants.kMaxAccelerationMeterPerSecondSquared)),
        drive::getLeftEncoderPosition,
        distance,
        (output, setpoint) ->
            drive.arcadeDrive(-(output + new SimpleMotorFeedforward(constants.kSVolts, constants.kVVoltMetersPerSecond).calculate(setpoint.velocity)), 0, false),
        drive);
    
    getController().setTolerance(constants.kDriveToleranceMeters, constants.kDriveVelocityToleranceMetersPerSecond);

  }


  @Override
  public void execute() {
    SmartDashboard.putNumber("Setpoint Velocity", getController().getSetpoint().velocity);
    SmartDashboard.putNumber("Actual Velocity", getController().getSetpoint().velocity + getController().getVelocityError());
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
