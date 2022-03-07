package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.TurnPID.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants.TurnPID;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleProfiled extends ProfiledPIDCommand {
  private DriveSubsystem m_drive;
  private TurnPID m_constants;

  public TurnToAngleProfiled(double goalAngle, DriveSubsystem drive, TurnPID constants) {
    super(
        new ProfiledPIDController(
            constants.kP,
            constants.kI,
            constants.kD,
            new TrapezoidProfile.Constraints(
                constants.kMaxVelocityDegreesPerSecond, constants.kMaxAccelerationDegreesPerSecondSquared)),
        drive::getHeading,
        goalAngle,
        (output, setpoint) ->
            drive.arcadeDrive(0, output + new SimpleMotorFeedforward(constants.kSVolts, constants.kVVoltDegreesPerSecond).calculate(setpoint.velocity), false),
        drive);

    m_drive = drive;
    m_constants = constants;

    // Make gyro values wrap around to avoid taking the long route to an angle
    getController().enableContinuousInput(-180, 180);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(constants.kTurnToleranceDeg, constants.kTurnRateToleranceDegPerS);
  }

  @Override
  public void initialize() {
    // Make sure to reset the heading before resetting the internal PID controller
    m_drive.resetHeading();
    super.initialize();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Setpoint Velocity", getController().getSetpoint().velocity);
    SmartDashboard.putNumber(
        "Current Velocity",
        super.getController().getSetpoint().velocity + getController().getVelocityError());

    super.execute();
  }

  @Override
  public boolean isFinished() {
    // The command finishes once the robot is done turning
    return getController().atGoal();
  }
}
