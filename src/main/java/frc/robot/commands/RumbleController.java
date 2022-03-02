// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// indexer logic using color sensor input

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Rumbles the controller at the specified intensity.
 * Call using .withTimeout()
 */

public class RumbleController extends CommandBase { 

    private XboxController m_controller;
    private double m_intensity;

    public RumbleController(XboxController controller, double intensity) {
        m_controller = controller;
        m_intensity = intensity;
    }
    
    @Override
    public void initialize() {
        m_controller.setRumble(RumbleType.kLeftRumble, m_intensity);
        m_controller.setRumble(RumbleType.kRightRumble, m_intensity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_controller.setRumble(RumbleType.kLeftRumble, 0);
        m_controller.setRumble(RumbleType.kRightRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
