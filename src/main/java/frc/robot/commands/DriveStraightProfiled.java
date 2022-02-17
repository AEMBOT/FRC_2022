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
            drive.arcadeDrive((output + m_feedforward.calculate(setpoint.velocity)), 0, false),
        drive);
    
    getController().setTolerance(kDriveToleranceMeters, kDriveVelocityToleranceMetersPerSecond);

    m_drive = drive;
  }

  @Override
  public void initialize() {
    // Make sure to reset the heading before resetting the internal PID controller
    m_drive.resetHeading();
    
 
    // For some reason, the odometry reset call from above isn't working
    // and the controller is re-initializing with the previous distance
    // it had before. Hacking it by manually doing what the caller is doing
    // (resetting the controller)
    //super.initialize();
    //System.out.print("Foo: ");
    //System.out.println(m_drive.getLeftEncoderPosition());
    super.getController().reset(0);
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
    return getController().atGoal();
  }
}
