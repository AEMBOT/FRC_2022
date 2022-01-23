package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import static frc.robot.Constants.DriveConstants.TurnPID.*;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleProfiled extends ProfiledPIDCommand {

    // private static final SimpleMotorFeedforward m_feedforward =
    //     new SimpleMotorFeedforward(kSVolts, kVVoltRadiansPerSecond);

    public TurnToAngleProfiled(double goalAngle, DriveSubsystem drive) {
        super(
            new ProfiledPIDController(kP, kI, kD,
                new TrapezoidProfile.Constraints(kMaxVelocityDegreesPerSecond, kMaxAccelerationDegreesPerSecondSquared)), 
            // TODO: A negative angle might be a problem, so I should probably correct for that
            () -> drive.getHeading(), goalAngle,
            // (output, setpoint) -> drive.arcadeDrive(0, output + m_feedforward.calculate(setpoint.velocity)), drive);
            (output, setpoint) -> drive.arcadeDrive(0, output), drive);
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
