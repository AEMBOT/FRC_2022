// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Gets values from a color sensor, and converts them to HSV. Accessors included to use for indexing purposes.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hardware.ClosedLoopSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class IndexerSubsystem extends SubsystemBase {
    //hardware constants
    private final I2C.Port PORT = I2C.Port.kOnboard;

    private final ColorSensorV3 m_sensor;

    private final ClosedLoopSparkMax m_lowerBelt;
    private final ClosedLoopSparkMax m_upperBelt;

    private boolean running = false;
    //creates new ExampleSubsystem
    public IndexerSubsystem(){
        m_sensor = new ColorSensorV3(PORT);

        m_lowerBelt = new ClosedLoopSparkMax(Constants.IntakeConstants.kIndexerUpperBottomBeltPort, MotorType.kBrushless);
        m_upperBelt = new ClosedLoopSparkMax(Constants.IntakeConstants.kIndexerTopBeltPort, MotorType.kBrushless);

        m_lowerBelt.follow(m_upperBelt);
    }

    @Override
    public void periodic(){
        //called once per scheduler run
        //readSensorHSV();
    }

    @Override
    public void simulationPeriodic() {
        //this method will be called 1 per scheduler during sim
    }

    /** Return the alliance that the loaded cargo belongs to 
     * TODO: Need to test this on the indexer
    */
    public Alliance getCargoAllianceColor(){
        double[] hsv = readSensorHSV();

        double hue = hsv[0];
        double sat = hsv[1];
        double val = hsv[2];

        //TODO: This will likely need to be updated with actual color values from the robot
        if( (hue > 200 && hue < 260) && (val > 50) && (sat > 75) ){
            return Alliance.Red;
        }
        //TODO: This will likely need to be updated with actual color values from the robot
        else if( (hue < 15 || hue > 350) && (val > 50) && (sat > 75) ){
            return Alliance.Blue;
        }
        else {
            return Alliance.Invalid;
        }
    }

    public String getColorString(){

        switch (getCargoAllianceColor()) {
            case Blue: return "BLUE";
            case Red: return "RED";
            default: return "NONE";
        }
    }

    public boolean ballDetectedAtEntry(){
        return true;
    }

    /** turns on exit belt */
    public void powerExitSide(double motorPower) {
        m_upperBelt.set(motorPower);
    }

    /** turns exit belts on or off */
    public void toggleExitSide(){
        if(!running) {
            powerExitSide(.5);
        }
        
        else {
            powerExitSide(0);
        }
    }

    private double[] readSensorHSV() {
        double red, blue, green;

        red = m_sensor.getRed();
        blue = m_sensor.getBlue();
        green = m_sensor.getGreen();
    
        //does math to make hsv
        red /= 255;
        blue /= 255;
        green /= 255;

        double cmax = Math.max(red, Math.max(green, blue));
        double cmin = Math.min(red, Math.min(green, blue));
        double diff = cmax - cmin;
        double h = -1;
        double s = -1;
                
        // if cmax and cmin are equal then h = 0
        if (cmax == cmin)
            h = 0;
 
        // if cmax equal r then compute h
        else if (cmax == red)
            h = (60 * ((green - blue) / diff) + 360) % 360;
 
        // if cmax equal g then compute h
        else if (cmax == green)
            h = (60 * ((blue - red) / diff) + 120) % 360;
 
        // if cmax equal b then compute h
        else if (cmax == blue)
            h = (60 * ((red - green) / diff) + 240) % 360;
 
        // if cmax equal zero
        if (cmax == 0)
            s = 0;
        else
            s = (diff / cmax) * 100;
 
        // compute v
        double v = cmax * 100;

        //updates dashboard with latest RGB and HSV readings
        SmartDashboard.putNumber("Red", red);
        SmartDashboard.putNumber("Blue", blue);
        SmartDashboard.putNumber("Green", green);

        SmartDashboard.putNumber("Hue", h);
        SmartDashboard.putNumber("Saturation", s);
        SmartDashboard.putNumber("Value", v);

        SmartDashboard.putString("Color:", getColorString());

        return new double[] {h,s,v};
    }
}