package frc.robot.commands.drive;

import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A command that uses PID control, feedforward, and motion profiling to drive the specified
 * distance.
 *
 * <p>This is untested but should work in theory™.
 */
public class DriveMeters extends TrapezoidProfileCommand {
  private DriveSubsystem m_drive;

  /**
   * Constructs a DriveMeters command to drive forward some distance while following a trapezoidal
   * motion profile.
   *
   * @param distance The distance to drive, in meters.
   * @param drive The robot's drive subsystem.
   */
  public DriveMeters(double distance, DrivetrainSubsystem drive) {
    super(
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecondSquared),

            // We want to have travelled the distance and end with a velocity of zero
            new TrapezoidProfile.State(distance, 0)),

        // Drive at the profile velocity using closed-loop control
        state -> drive.tankDriveVelocities(state.velocity, state.velocity),
        drive);

    m_drive = drive;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    // Reset previous wheel velocities for acceleration calculations
    m_drive.resetPreviousVelocities();
  }
}
