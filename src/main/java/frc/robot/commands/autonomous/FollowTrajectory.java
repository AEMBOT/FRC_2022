package frc.robot.commands.autonomous;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.DrivetrainConstants.Ramsete.*;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/** A command that has the robot follow a pre-generated trajectory. */
public class FollowTrajectory extends RamseteCommand {
  private DrivetrainSubsystem m_drive;
  private Trajectory m_trajectory;

  private static final DifferentialDriveKinematics s_kinematics =
      new DifferentialDriveKinematics(kEffectiveTrackWidth);

  /**
   * Constructs a FollowTrajectory to have the robot drive along a trajectory.
   *
   * @param drive The robot's drive subsystem
   * @param trajectory The trajectory for the robot to follow
   */
  public FollowTrajectory(DrivetrainSubsystem drive, Trajectory trajectory) {
    super(
        trajectory,
        drive::getPose,

        // Ramsete controller for staying on the trajectory
        new RamseteController(kRamseteB, kRamseteZeta),

        // Kinematics (for handling trajectory curvature)
        s_kinematics,

        // Used to drive the wheels at the correct velocities
        drive::tankDriveVelocities,
        drive);

    m_drive = drive;
    m_trajectory = trajectory;
  }

  @Override
  public void initialize() {
    super.initialize();

    // This is necessary to track the trajectory properly
    m_drive.resetOdometryAndEncoders(m_trajectory.getInitialPose());
  }
}
