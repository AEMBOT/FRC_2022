package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.comman

//command list for autopathing. doesn't do anything yet
public class AutonomousPathing extends SequentialCommandGroup {
    public AutonomousPathing(DriveSubsystem drive) { 
        addCommands(
            //Starting position is determined by jig - see discord for specifics

            //back up into ball nearest hub
            new InstantCommand(() -> drive.driveInches(-52), drive),    
            //--intake here
            
            //drive forwards to line up shot
            new InstantCommand(() -> drive.driveInches(84), drive),
            //--shoot here
            
            //rotate to align with 2 far balls
            new InstantCommand(() -> drive.turnDegrees(75), drive),
            
            //drive back to first ball
            new InstantCommand(() -> drive.driveInches(-87), drive),
            //--intake here
            
            //drive back to second ball
            new InstantCommand(() -> drive.driveInches(-157), drive),
            //--intake here

            //return to hub, shorten as needed          
            new InstantCommand(() -> drive.driveInches(244), drive),
            
            //line up shot, shoot
            new InstantCommand(() -> drive.turnDegrees(105), drive)
            //--shoot here

        );//end command list
    }
}