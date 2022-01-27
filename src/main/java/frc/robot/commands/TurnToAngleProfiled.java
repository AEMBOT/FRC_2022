package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import static frc.robot.Constants.DriveConstants.TurnPID.*;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleProfiled extends ProfiledPIDCommand {

    private static final SimpleMotorFeedforward m_feedforward =
        new SimpleMotorFeedforward(kSVolts, kVVoltDegreesPerSecond);

    public TurnToAngleProfiled(double goalAngle, DriveSubsystem drive) {
        super(
            new ProfiledPIDController(kP, kI, kD,
                new TrapezoidProfile.Constraints(kMaxVelocityDegreesPerSecond, kMaxAccelerationDegreesPerSecondSquared)), 
                () -> drive.getHeading(), goalAngle,
            (output, setpoint) -> drive.arcadeDrive(0, output + m_feedforward.calculate(setpoint.velocity)), drive);

        // Make gyro values wrap around to avoid taking the long route to an angle
        getController().enableContinuousInput(-180, 180);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
      .setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);

    //drive.resetHeading();
        
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
