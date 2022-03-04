package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveStraightSmart;
import frc.robot.subsystems.DriveSubsystem;


//command list for autopathing. doesn't do anything yet
public class AutonomousPathing extends SequentialCommandGroup {
    public AutonomousPathing(DriveSubsystem drive) { 
        addCommands(
            
            new DriveStraightSmart(-80/12.0, drive),
            new DriveStraightSmart(80/12.0, drive),
            //shoot
            //turn 70 degrees clockwise
            new DriveStraightSmart(-275/12.0, drive),
            //intake
            new DriveStraightSmart(275/12.0, drive)
            //shoot twice
            );//end command list
    }
}
