package frc.robot.commands.autonomous;

import static frc.robot.Constants.DrivetrainConstants.Ramsete.*;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowTrajectory extends RamseteCommand {
  private DrivetrainSubsystem m_drive;
  private Trajectory m_trajectory;

  public FollowTrajectory(DrivetrainSubsystem drive, Trajectory trajectory) {
    super(
        trajectory,
        drive::getPose,
        new RamseteController(kRamseteB, kRamseteZeta),
        kDriveKinematics,
        drive::driveAtVelocity,
        drive);

    m_drive = drive;
    m_trajectory = trajectory;
  }

  @Override
  public void initialize() {
    super.initialize();
    m_drive.resetOdometryAndEncoders(m_trajectory.getInitialPose());
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    // Stop the drivetrain motors after finishing the path
    m_drive.stopMotors();
  }
}
