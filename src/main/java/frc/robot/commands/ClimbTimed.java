// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.BooleanSupplier;

public class ClimbTimed extends SequentialCommandGroup {

  public ClimbTimed(ClimberSubsystem climber, BooleanSupplier condition_button_press) {

    addCommands(
        // Default state
        new InstantCommand(climber::setRetracting, climber),
        new InstantCommand(climber::verticalMainCylinders, climber),
        new WaitUntilCommand(condition_button_press),

        // send hooks up
        new InstantCommand(climber::setExtending, climber),
        new WaitCommand(1.5),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(3),

        new WaitUntilCommand(condition_button_press),

        // retract hooks
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(1.5),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(3),

        new WaitUntilCommand(condition_button_press),

        // send hooks up
        new InstantCommand(climber::setExtending, climber),

        // ---attach chin here
        // ---robot should now be hanging from first bar
        // But do it slowly-ish to prevent bouncing!
        new WaitCommand(0.15),
        new InstantCommand(climber::setCoasting, climber),

        new WaitUntilCommand(condition_button_press),

        // angle hooks
        // start angling a little after main cylinders start extending (want the hooks to disengage)
        new InstantCommand(climber::angleMainCylinders, climber),

        new WaitUntilCommand(condition_button_press),

        // Choose to not angle and extend at the same time for right now
        new InstantCommand(climber::setExtending, climber),

        // hooks are offset, aligned for second bar

        // continue to send main cylinders up (to full length)
        new WaitCommand(0.65),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(0.7),

        new WaitUntilCommand(condition_button_press),

        // adjust angle to attach hooks to second bar
        new InstantCommand(climber::verticalMainCylinders, climber),
        // give them a little time to vertical enough

        new WaitUntilCommand(condition_button_press),

        // retract hooks halfway
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(1.0),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(0.35),

        new WaitUntilCommand(condition_button_press),

        // retract hooks fully
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(1.5),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(0.35),

        /// REPEATED FROM ABOVE, except for slight "wait" differences
        new WaitUntilCommand(condition_button_press),

        // send hooks up
        new InstantCommand(climber::setExtending, climber),

        // ---attach chin here
        // ---robot should now be hanging from first bar
        // But do it slowly-ish to prevent bouncing!
        new WaitCommand(0.4),
        new InstantCommand(climber::setCoasting, climber),
        new WaitUntilCommand(condition_button_press),

        // angle hooks
        // start angling a little after main cylinders start extending (want the hooks to disengage)
        new InstantCommand(climber::angleMainCylinders, climber),
        new WaitUntilCommand(condition_button_press),

        // Choose to not angle and extend at the same time for right now
        new InstantCommand(climber::setExtending, climber),

          // continue to send main cylinders up (to full length)
        new WaitCommand(1.8),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(0.3),

        // Wait for swinging to stop?? Confirm next action with button press
        new WaitUntilCommand(condition_button_press),

        // adjust angle to attach hooks to second bar
        new InstantCommand(climber::verticalMainCylinders, climber),
        // give them a little time to extend enough

        new WaitUntilCommand(condition_button_press),

        // retract hooks halfway
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(0.75),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(0.15),

        // retract hooks fully
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(2.67),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(0.33)); // end command list
  }
}
