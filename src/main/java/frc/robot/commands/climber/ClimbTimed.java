// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;

public class ClimbTimed extends SequentialCommandGroup {

  public ClimbTimed(
      ClimberSubsystem climber, IntakeSubsystem intake, BooleanSupplier condition_button_press) {

    addRequirements(climber, intake);

    addCommands(
        // Default state
        new InstantCommand(climber::setRetracting, climber),
        new InstantCommand(climber::verticalMainCylinders, climber),
        new WaitUntilCommand(condition_button_press),

        // Take over intake and lower it for the rest of the climb
        // new RunIntakeWinchToPosition(intake, Constants.IntakeConstants.kWinchLoweredPosition),

        // send hooks up
        new InstantCommand(climber::setExtending, climber),
        new WaitCommand(1.0),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(2),
        new WaitUntilCommand(condition_button_press),

        // retract hooks
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(2.2),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(1),
        new WaitUntilCommand(condition_button_press),

        // send hooks up
        new InstantCommand(climber::setExtending, climber),

        // ---attach chin here
        // ---robot should now be hanging from first bar
        // But do it slowly-ish to prevent bouncing!
        new WaitCommand(0.25),

        //////////////////////
        // REMOVE ME AFTER INITIAL TESTING
        new InstantCommand(climber::setCoasting, climber),
        new WaitUntilCommand(condition_button_press),
        //////////////////////

        // angle hooks
        // start angling a little after main cylinders start extending (want the hooks to disengage)
        new InstantCommand(climber::angleMainCylinders, climber),
        new WaitUntilCommand(condition_button_press),

        // Choose to not angle and extend at the same time for right now
        new InstantCommand(climber::setExtending, climber),

        // hooks are offset, aligned for second bar

        // continue to send main cylinders up (to full length)
        new WaitCommand(0.85),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(2.0),
        new WaitUntilCommand(condition_button_press),

        // adjust angle to attach hooks to second bar
        new InstantCommand(climber::verticalMainCylinders, climber),
        new WaitUntilCommand(condition_button_press),

        // retract hooks all the way
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(1.0),
        new InstantCommand(climber::setCoasting, climber),
        new WaitCommand(2.5),
        new InstantCommand(climber::setRetracting, climber),
        new WaitCommand(1.5),
        new InstantCommand(climber::setCoasting, climber),

        /// REPEATED FROM ABOVE, except for slight "wait" differences
        new WaitUntilCommand(condition_button_press),

        // send hooks up
        new InstantCommand(climber::setExtending, climber),

        // ---attach chin here
        // ---robot should now be hanging from first bar
        // But do it slowly-ish to prevent bouncing!
        new WaitCommand(0.4),
        //////////////////
        // REMOVE ME
        new InstantCommand(climber::setCoasting, climber),
        new WaitUntilCommand(condition_button_press),
        //////////////////

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
        new WaitUntilCommand(condition_button_press),

        // retract hooks fully
        new InstantCommand(climber::setRetracting, climber));

    // Maybe add a coast here so we don't run out of air?
  }
}
