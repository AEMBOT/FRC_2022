/*
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.drive.TurnToAngleProfiled;
import frc.robot.subsystems.DriveSubsystem;

public class TxToTurn {
    private DriveSubsystem m_drive;
    private double m_power;

    float tx = (float) 
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    private TxToTurn(double power, DriveSubsystem drive){
        m_drive = drive;
        m_power = power;
    }

    private void Turn(DriveSubsystem drive){
        TurnToAngleProfiled(tx, m_drive);
    }
    
}
*/