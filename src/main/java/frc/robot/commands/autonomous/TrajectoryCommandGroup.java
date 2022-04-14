package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.ListIterator;
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

  // A map of commands that haven't executed
  // This is kept to avoid triggering a command multiple times
  private final Map<Command, PositionTrigger> m_untriggeredCommands = new HashMap<>();

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
    m_untriggeredCommands.clear();
    m_untriggeredCommands.putAll(m_allCommands);

    // Clear the executing command list
    m_executingCommands.clear();

    // Initialize the trajectory following command
    m_trajectoryCommand.initialize();
  }

  @Override
  public void execute() {
    // Drive along the trajectory
    m_trajectoryCommand.execute();

    // A ListIterator allows for the modification of a list while iterating over it
    // This avoids a ConcurrentModificationException
    ListIterator<Command> executing = m_executingCommands.listIterator();

    // Execute the previously triggered commands
    while (executing.hasNext()) {
      Command command = executing.next();

      command.execute();

      // Check if the command is finished
      if (command.isFinished()) {
        // End the command, which finished naturally if isFinished() returns true
        command.end(false);

        // Remove the command from the currently executing list since it's finished executing
        executing.remove();
      }
    }

    // Get the robot's current position on the field as a Translation2d
    Translation2d currentPosition = m_drive.getPose().getTranslation();

    // Check if any commands should be triggered by the robot's position
    for (Map.Entry<Command, PositionTrigger> poseCommand : m_untriggeredCommands.entrySet()) {
      Command command = poseCommand.getKey();
      PositionTrigger goal = poseCommand.getValue();

      // Execute the command if it just got triggered, or has been previously
      if (currentPosition.getDistance(goal.position) <= goal.tolerance) {
        // Initialize the command
        command.initialize();

        // Remove the command from the untriggered map
        m_untriggeredCommands.remove(command);

        // Add the command to the currently executing list
        m_executingCommands.add(command);
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

  /**
   * Don't use this method to add commands; use {@link #addCommands(Map)} instead.
   *
   * @throws IllegalArgumentException Whenever this overload is called
   */
  @Override
  public void addCommands(Command... commands) {
    throw new IllegalArgumentException(
        "Commands can't be added to a TrajectoryCommandGroup without position triggers.");
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

      // Throw an exception if two commands share a subsystem dependency
      if (!Collections.disjoint(m_requirements, command.getRequirements())) {
        throw new IllegalArgumentException(
            "Commands can't require the same subsystem in a TrajectoryCommandGroup. Use ProxyScheduleCommands if this is necessary.");
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
