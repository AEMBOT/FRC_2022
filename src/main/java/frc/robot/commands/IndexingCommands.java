// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// indexer logic using color sensor input

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utilities.TimedRumble;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class IndexingCommands extends CommandBase { 
    private final IndexerSubsystem m_index;
    private String teamColor;
    private XboxController m_controller;

    public IndexingCommands(IndexerSubsystem subsystem, XboxController controller){
        m_index = subsystem;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
        {
            teamColor = "BLUE";
        }
        else
        {
            teamColor = "RED";
        }
        m_controller = controller;
    }
      
    
    
    @Override
    public void initialize() {
        if (m_index.getColorString() == teamColor){
            if (m_index.ballDetectedAtEntry()){

            }
        }
        else
        {
            if(!m_index.ballDetectedAtEntry()){
                // Provide a slight rumble jolt to the controller
                //                    .25 seconds, .5 power
                new TimedRumble(m_controller, 0.25, .5);
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
