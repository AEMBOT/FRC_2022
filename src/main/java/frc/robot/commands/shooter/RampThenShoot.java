package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TeleOpShooter;
import frc.robot.commands.indexer.RunUpperIndexer;
import frc.robot.commands.utilities.Noop;
import frc.robot.commands.utilities.TimedRumble;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimeLightTargeting;
import frc.robot.subsystems.ShooterSubsystem;

public class RampThenShoot extends SequentialCommandGroup {
    private LimeLightTargeting m_limelight;

    public RampThenShoot(IndexerSubsystem indexer, ShooterSubsystem shooter, LimeLightTargeting limelight, XboxController driverController) {
        m_limelight = limelight;
        addCommands(
            // Turn on the limelight LED and allow for some time for that to actually happen
            // new InstantCommand(limelight::turnOnLED),
            // new WaitCommand(0.1),

            // Ramp up the shooter to the desired power, rumbling the driver controller if there's no detected target
            new ParallelCommandGroup(
                new RampShooter(shooter).withTimeout(0.33),
                new ConditionalCommand(
                    new Noop(), new TimedRumble(driverController, 0.25, 0.5), 
                    limelight::hasValidTarget)
            ),

            // Run the shooter and upper indexer belt at the same time after ramping up the shooter power
            new ParallelCommandGroup(
                new TeleOpShooter(shooter),
                new RunUpperIndexer(indexer)
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Turn off the limelight LED after canceling all of the commands
        // m_limelight.turnOffLED();
    }
}
