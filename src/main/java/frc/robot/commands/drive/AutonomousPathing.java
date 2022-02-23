package frc.robot.commands.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;


//command list for autopathing. doesn't do anything yet
public class AutonomousPathing extends SequentialCommandGroup {
    public AutonomousPathing(DriveSubsystem drive) { 
        addCommands(
            
            new DriveStraightSmart(Units.inchesToMeters(-80), drive),
            new DriveStraightSmart(Units.inchesToMeters(80), drive),
            //shoot
            //turn 70 degrees clockwise
            new DriveStraightSmart(Units.inchesToMeters(-275) , drive),
            //intake
            new DriveStraightSmart(Units.inchesToMeters(275), drive)
            //shoot twice
            );//end command list
    }
}
