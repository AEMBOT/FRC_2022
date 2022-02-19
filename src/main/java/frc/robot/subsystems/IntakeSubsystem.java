package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.ClosedLoopSparkMax;

public class IntakeSubsystem extends SubsystemBase{
    // TODO: move these later 
    private final int kLiftLeftPort = 7;
    private final int kLiftRightPort = 8;
    private final int kRollerPort = 9;

    // motor controllers
    private final CANSparkMax liftLeft = new CANSparkMax(kLiftLeftPort, MotorType.kBrushless);
    private final CANSparkMax liftRight = new CANSparkMax(kLiftRightPort, MotorType.kBrushless);
    
    private final ClosedLoopSparkMax roller = new ClosedLoopSparkMax(kRollerPort, MotorType.kBrushless);

    // whether the subsystem is successfully homed to its max point
    private boolean homingComplete = false;

    // The encoder positions for the lowest and highest allowed
    private double m_lowestAllowedPosition;
    private double m_highestAllowedPosition;

    private boolean rollerRunning = true;

    // motor geared 125:1 -> 24:72 gearing
    public IntakeSubsystem() {

        // TODO: set position conversion factor
        double factor = 1;

        // I value needs to be nonzero in order for closed loop PID to work
        roller.kI(000001);

        // set the left side to follow the right side, invert=false
        liftLeft.follow(liftRight, true);

        // set the position conversion factors for the lift encoders
        liftRight.getEncoder().setPositionConversionFactor(factor);
    }

    /** set the intake mechanism to run at the target RPM */
    public void setRPM(double rpm) {
        final double kGearRatio = 1/12;
        roller.setVelocity(rpm * kGearRatio);
    }

    private void runRollerAtMaxPower() {
        roller.set(1.0);
    }

    public void toggleRoller() {
        if (rollerRunning) {
            roller.set(0);
            rollerRunning = false;
        }

        else {
            runRollerAtMaxPower();
            rollerRunning = true;
        }
    }

    /** sets the range of motion for the intake lift. */
    public void setHome(double min, double max) {

        if (homingComplete) {
            System.out.println("Intake being re-homed!");
        }

        m_lowestAllowedPosition = min;
        m_highestAllowedPosition = max;

        homingComplete = true;
    }

    public boolean isStopped() {
        final double minVelocityTolerance = 0;
        return (Math.abs(liftLeft.getEncoder().getVelocity()) <= 0) || (Math.abs(liftRight.getEncoder().getVelocity()) <= 0);
    }

    public void setLiftPower(double power) {
        liftRight.set(power);
    }

    /** gets the encoder position of the left lift */
    public double getLiftPosition() {
        return liftRight.getEncoder().getPosition();
    }
}
