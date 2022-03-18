package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.LEDMode;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithHubSmart extends SequentialCommandGroup {
  private Limelight m_limelight;

  public AlignWithHubSmart(Limelight limelight, DriveSubsystem drive) {
    addCommands(
        // Allow some time for the Limelight LEDs to actually turn on
        new WaitCommand(0.2),

        // Turn towards the hub
        new TurnToAngleSmart(() -> -limelight.getX(), drive).withTimeout(0.5));
  }

  @Override
  public void initialize() {
    super.initialize();
    m_limelight.setLEDMode(LEDMode.On);
  }
}
