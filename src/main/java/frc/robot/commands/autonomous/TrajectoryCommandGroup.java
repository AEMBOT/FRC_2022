package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A command group that executes commands while driving along a trajectory.
 *
 * @author Zane Othman-Gomez
 */
public class TrajectoryCommandGroup extends CommandGroupBase {
  private final RamseteCommand m_trajectoryCommand;
  private final DrivetrainSubsystem m_drive;

  // A map of every command that is part of this group
  private final Map<Command, PositionTrigger> m_allCommands = new HashMap<>();

  // A map of commands that haven't executed (or finished executing)
  // This is kept to avoid triggering a command multiple times
  private final Map<Command, PositionTrigger> m_unfinishedCommands = new HashMap<>();

  // A list of currently executing commands (for ending purposes)
  private final List<Command> m_executingCommands = new ArrayList<>();

  public TrajectoryCommandGroup(
      Trajectory trajectory, DrivetrainSubsystem drive, Map<Command, PositionTrigger> commands) {
    // Create a command to follow the provided trajectory and inherit its requirements
    m_trajectoryCommand = new FollowTrajectory(drive, trajectory);
    m_requirements.addAll(m_trajectoryCommand.getRequirements());

    // The trajectory command shouldn't be able to be used elsewhere (although this is probably
    // unnecessary since it's instantiated here)
    requireUngrouped(m_trajectoryCommand);

    // Store a reference to the drivetrain subsystem for fetching the robot's pose
    m_drive = drive;

    // Add the commands to this command group
    addCommands(commands);
  }

  @Override
  public void initialize() {
    // Clear and repopulate the unfinished command map
    m_unfinishedCommands.clear();
    m_unfinishedCommands.putAll(m_allCommands);

    // Clear the executing command list
    m_executingCommands.clear();
  }

  @Override
  public void execute() {
    // Drive along the trajectory
    m_trajectoryCommand.execute();

    // Get the robot's current position on the field as a Translation2d
    Translation2d currentPosition = m_drive.getPose().getTranslation();

    // Iterate over all unfinished commands in the group
    for (Map.Entry<Command, PositionTrigger> poseCommand : m_unfinishedCommands.entrySet()) {
      Command command = poseCommand.getKey();
      PositionTrigger goal = poseCommand.getValue();

      boolean currentlyExecuting = m_executingCommands.contains(command);

      // Execute the command if it just got triggered, or has been previously
      if (currentlyExecuting || currentPosition.getDistance(goal.position) <= goal.tolerance) {

        // Add the command to the currently executing list if it's not there already
        if (!currentlyExecuting) {
          m_executingCommands.add(command);
        }

        // Execute the command
        command.execute();

        // Check if the command is finished
        if (command.isFinished()) {
          // End the command, which finished naturally if isFinished() returns true
          command.end(false);

          // Remove the command from the unfinished map & currently executing list
          m_unfinishedCommands.remove(command);
          m_executingCommands.remove(command);
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // End the trajectory command
    m_trajectoryCommand.end(interrupted);

    // End all other commands that were executing when this command finished/was interrupted
    for (Command command : m_executingCommands) {
      command.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    // Finishes when the robot has finished driving along the trajectory
    return m_trajectoryCommand.isFinished();
  }

  @Override
  public void addCommands(Command... commands) {
    throw new IllegalArgumentException(
        "Commands can't be added to a TrajectoryCommandGroup without goal poses.");
  }

  /**
   * Adds the commands to this command group. They will be triggered near their supplied position
   * triggers.
   *
   * @param commands The map of commands to add to this group.
   */
  public void addCommands(Map<Command, PositionTrigger> commands) {
    // Commands shouldn't be used elsewhere
    requireUngrouped(commands.keySet());

    for (Map.Entry<Command, PositionTrigger> poseCommand : commands.entrySet()) {
      Command command = poseCommand.getKey();

      // Throw an exception if the command depends on the drive subsystem, since
      // two commands can't use a subsystem at the same time
      if (command.hasRequirement(m_drive)) {
        throw new IllegalArgumentException(
            "Command "
                + command.getName()
                + " can't require the drive subsystem in a TrajectoryCommandGroup.");
      }

      // Add the command to this group's map along with its position trigger
      m_allCommands.put(command, poseCommand.getValue());

      // Inherit the subsystem requirements of the added command if there are no conflicts
      m_requirements.addAll(command.getRequirements());
    }
  }

  /**
   * A class that represents an area in which to trigger a command during a {@link
   * TrajectoryCommandGroup}.
   */
  public static class PositionTrigger {
    // TODO: This can probably become a record class when WPILib updates JDKs
    public final Translation2d position;
    public final double tolerance;

    /**
     * Constructs a PositionTrigger.
     *
     * @param position The position to trigger the command near.
     * @param tolerance How close the robot needs to be to the desired position to activate the
     *     command.
     */
    public PositionTrigger(Translation2d position, double tolerance) {
      this.position = position;
      this.tolerance = tolerance;
    }
  }
}
