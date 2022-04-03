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

public class ClimbManual extends SequentialCommandGroup {

  public ClimbManual(ClimberSubsystem climber, BooleanSupplier condition_button_press) {

    // Revert if needed. Include a few button presses still for safety during testing
    BooleanSupplier condition = condition_button_press;

    addCommands(
        // Default state
        new InstantCommand(climber::setRetracting, climber),
        new InstantCommand(climber::setPistonsVertical, climber),
        new WaitUntilCommand(condition_button_press),

        // send hooks up
        new InstantCommand(climber::extendArms, climber),
        new WaitUntilCommand(condition),
        new InstantCommand(climber::cutOffPressure, climber),
        // retract hooks
        new WaitUntilCommand(condition),
        new InstantCommand(climber::setRetracting, climber),
        new WaitUntilCommand(condition),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition),

        // send hooks up
        new InstantCommand(climber::extendArms, climber),

        // ---attach chin here
        // ---robot should now be hanging from first bar
        // But do it slowly-ish to prevent bouncing!
        new WaitCommand(0.15),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition_button_press),

        // angle hooks
        // start angling a little after main cylinders start extending (want the hooks to disengage)
        new InstantCommand(climber::setPistonsAngled, climber),
        new WaitUntilCommand(condition_button_press),

        // Choose to not angle and extend at the same time for right now
        new InstantCommand(climber::extendArms, climber),

        // hooks are offset, aligned for second bar

        // continue to send main cylinders up (to full length)
        new WaitUntilCommand(condition),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition),

        // adjust angle to attatch hooks to second bar
        new InstantCommand(climber::setPistonsVertical, climber),
        // give them a little time to extend enough
        new WaitUntilCommand(condition),

        // retract hooks halfway
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(0.75),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition),
        // retract hooks fully
        new InstantCommand(climber::setRetracting, climber),
        new WaitUntilCommand(condition),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition),

        /// REPEATED FROM ABOVE, except for slight "wait" differences

        // send hooks up
        new InstantCommand(climber::extendArms, climber),

        // ---attach chin here
        // ---robot should now be hanging from first bar
        // But do it slowly-ish to prevent bouncing!
        new WaitCommand(0.4),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition_button_press),

        // angle hooks
        // start angling a little after main cylinders start extending (want the hooks to disengage)
        new InstantCommand(climber::setPistonsAngled, climber),
        new WaitUntilCommand(condition_button_press),

        // Choose to not angle and extend at the same time for right now
        new InstantCommand(climber::extendArms, climber),

        // continue to send main cylinders up (to full length)
        new WaitUntilCommand(condition),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition),

        // Wait for swinging to stop?? Confirm next action with button press
        new WaitUntilCommand(condition_button_press),

        // adjust angle to attatch hooks to second bar
        new InstantCommand(climber::setPistonsVertical, climber),
        // give them a little time to extend enough
        new WaitUntilCommand(condition),

        // retract hooks halfway
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(0.75),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition),

        // retract hooks fully
        new InstantCommand(climber::setRetracting, climber),
        new WaitUntilCommand(condition),
        new InstantCommand(climber::cutOffPressure, climber),
        new WaitUntilCommand(condition)); // end command list
  }
}
