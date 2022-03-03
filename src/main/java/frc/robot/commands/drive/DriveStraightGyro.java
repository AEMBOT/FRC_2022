package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.StraightPID.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightGyro extends CommandBase {
  private final Timer m_timer = new Timer();
  private final PIDController m_gyroPID = new PIDController(0.01, 0, 0);

  private TrapezoidProfile m_profile;
  private double m_distance;
  private DriveSubsystem m_drive;

  public DriveStraightGyro(double distance, double offsetAngle, DriveSubsystem drive) {
    // Create a trapezoidal motion profile
    m_profile =
        new TrapezoidProfile(
            // TODO: Tune these constraints, since they were arbitrary
            new TrapezoidProfile.Constraints(1.5, 1.0), new TrapezoidProfile.State(distance, 0));

    // Set the gyro PID setpoint to the desired offset angle
    m_gyroPID.setSetpoint(offsetAngle);

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

    // Start the timer for profiling purposes
    m_timer.start();
  }

  @Override
  public void execute() {
    // Calculate the setpoint velocity from the motion profile
    double profileVelocity = m_profile.calculate(m_timer.get()).velocity / kRPMToMetersPerSecond;
    SmartDashboard.putNumber("DriveStraightGyro Velocity", profileVelocity);

    // Use a PID controller to adjust the velocities to ensure the robot drives at the right angle
    double angleAdjustment = m_gyroPID.calculate(m_drive.getHeading());
    m_drive.driveAtVelocity(profileVelocity - angleAdjustment, profileVelocity + angleAdjustment);
  }

  @Override
  public void end(boolean _interrupted) {
    // Stop the motors
    m_drive.driveAtVelocity(0, 0);

    // Stop and reset the timer
    m_timer.stop();
    m_timer.reset();
  }

  @Override
  public boolean isFinished() {
    return m_drive.smartMotionAtGoal(m_distance);
  }
}
