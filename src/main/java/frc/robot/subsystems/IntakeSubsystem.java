package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.ClosedLoopSparkMax;

public class IntakeSubsystem extends SubsystemBase{
    //TODO: move these later 
    private final int kliftLeftPort = 7;
    private final int kliftRightPort = 8;
    private final int kRollerPort = 9;

    public IntakeSubsystem(){
        roller.kI(000001);
        liftLeft.follow(liftRight,false);
    }

    private final CANSparkMax liftLeft = new CANSparkMax(kliftLeftPort, MotorType.kBrushless);
    private final CANSparkMax liftRight = new CANSparkMax(kliftRightPort, MotorType.kBrushless);
    private final ClosedLoopSparkMax roller = new ClosedLoopSparkMax(kRollerPort, MotorType.kBrushless);

    public void setRPM(double rpm){
        roller.setVelocity(rpm);
    }

    public void setAngle(double angle){

    }

    public void calibrate(){
        
    }

    public void setPower(double power){
        liftRight.set(power);
    }
}
