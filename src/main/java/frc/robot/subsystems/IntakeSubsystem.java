package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hardware.ClosedLoopSparkMax;

public class IntakeSubsystem extends SubsystemBase{

    // motor controllers
    private final CANSparkMax liftLeft = new CANSparkMax(Constants.IntakeConstants.kLiftLeftPort, MotorType.kBrushless);
    private final CANSparkMax liftRight = new CANSparkMax(Constants.IntakeConstants.kLiftRightPort, MotorType.kBrushless);
    
    private final ClosedLoopSparkMax intakeRoller = new ClosedLoopSparkMax(Constants.IntakeConstants.kRollerPort, MotorType.kBrushless);
    private final ClosedLoopSparkMax indexEntryRoller = new ClosedLoopSparkMax(Constants.IntakeConstants.kIndexerLowerBottomBeltPort, MotorType.kBrushless);

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
        intakeRoller.kI(000001);

        // upper/inner roller follows the outside
        indexEntryRoller.follow(intakeRoller);

        // set the left side to follow the right side, invert=false
        liftLeft.follow(liftRight, true);

        // set the position conversion factors for the lift encoders
        liftRight.getEncoder().setPositionConversionFactor(factor);
    }

    /** set the intake mechanism to run at the target RPM */
    public void setRPM(double rpm) {
        final double kGearRatio = 1/12;
        intakeRoller.setVelocity(rpm * kGearRatio);
    }

    private void runRollerAtMaxPower() {
        intakeRoller.set(0.5);
    }

    public void toggleRoller() {
        if (rollerRunning) {
            intakeRoller.set(0);
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
