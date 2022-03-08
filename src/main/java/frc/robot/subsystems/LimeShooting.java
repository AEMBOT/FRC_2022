package frc.robot.subsystems;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimeLightTargeting;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.DriveSubsystem;


public class LimeShooting extends SubsystemBase {
    private LimeLightTargeting m_lime;
    private ShooterSubsystem m_shoot;
    private DriveSubsystem m_drive;
    private TurnToAngleProfiled m_turn;
    
    public LimeShooting(DriveSubsystem drive, LimeLightTargeting lime){
        m_drive = drive;
        m_lime = lime;

        if (m_lime.hasValidTarget()){
            //TODO: replace following line with correct turn method
            m_turn.TurnToAngleProfiled(m_lime.getX(), m_drive);
            m_shoot.shootFlywheels(m_lime.getDistance());
        }
        else{
            //rumble controller
        }
    }

}
