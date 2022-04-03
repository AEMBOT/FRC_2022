// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.BooleanSupplier;

/**
 * A command that progresses through a mid-high-traversal climb sequence. Advancement is done via a
 * button press, but extension/retraction timings are handled programmatically.
 */
public class ClimbTimed extends SequentialCommandGroup {
  /**
   * Constructs a ClimbTimed command, which progresses through a mid-high-traversal climb sequence
   * using button presses.
   *
   * @param climber The robot's climber subsystem
   * @param trigger_advance The method to trigger an advance in the climbing sequence (normally a
   *     button press)
   */
  public ClimbTimed(ClimberSubsystem climber, BooleanSupplier trigger_advance) {
    // NOTE: The pressure cutoffs are done to conserve air, which allows us to reach traversal
    addCommands(
        // Default state (arms retracted & vertical)
        new InstantCommand(climber::setRetracting, climber),
        new InstantCommand(climber::setPistonsVertical, climber),

        // MID CLIMB

        // Fully extend arms, cutting off air early to conserve it
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::extendArms, climber),
        new WaitCommand(1),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitCommand(2),

        // Retract the hooks, presumably after the driver positions the bot correctly
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(2.2),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitCommand(1),

        // HIGH CLIMB

        // Extend the arms a bit to let go with the piston hooks (acrylic ones should still be on)
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::extendArms, climber),
        new WaitCommand(0.25),
        new InstantCommand(climber::cutOffPressure, climber),

        // Angle the arms to allow them to hook on to the high bar
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::setPistonsAngled, climber),

        // Extend the arms (the driver should do this after they are angled properly)
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::extendArms, climber),
        new WaitCommand(0.85),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitCommand(2.0),

        // Angle the arms vertically again, making them hit the bar
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::setPistonsVertical, climber),

        // Retract the hooks, grabbing onto the high bar
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(1.0),
        new InstantCommand(climber::cutOffPressure, climber),

        // This is done to avoid only hooking on with one side
        new WaitCommand(2.5),
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(1.5),
        new InstantCommand(climber::cutOffPressure, climber),

        // TRAVERSAL CLIMB

        // Unhook the pistons, leaving the acrylic hooks in place
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::extendArms, climber),
        new WaitCommand(0.4),
        new InstantCommand(climber::cutOffPressure, climber),

        // Angle the climber arms to allow them to hook onto the traversal bar
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::setPistonsAngled, climber),

        // Extend the arms
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::extendArms, climber),
        new WaitCommand(1.8),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitCommand(0.3),

        // Return the climber arms to a vertical(ish) state so they hit the traversal bar
        // Note that this has to be timed correctly, as otherwise they won't hook on properly
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::setPistonsVertical, climber),

        // Retract the hooks without a cutoff, since timings are inconsistent with so little air
        new WaitUntilCommand(trigger_advance),
        new InstantCommand(climber::setRetracting, climber)

        // ???
        // Profit
        );
  }
}
