// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Gets values from a color sensor, and converts them to HSV. Accessors included to use for indexing purposes.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;


public class IndexerSubsystem extends SubsystemBase {
    //hardware constants
    private final I2C.Port PORT = I2C.Port.kOnboard;

    private ColorSensorV3 m_sensor;

    //last colors recieved in RGB
    private double m_red;
    private double m_blu;
    private double m_gre;

    private double m_hue;
    private double m_sat;
    private double m_val;

    //creates new ExampleSubsystem
    public IndexerSubsystem(){
        m_sensor = new ColorSensorV3(PORT);
    }

    @Override
    public void periodic(){
        //called once per scheduler run
        m_red = m_sensor.getRed();
        m_blu = m_sensor.getBlue();
        m_gre = m_sensor.getGreen();
    
        //does math to make hsv
        m_red = m_red / 255;
        m_blu = m_blu / 255;
        m_gre = m_gre / 255;

        double cmax = Math.max(m_red, Math.max(m_gre, m_blu));
        double cmin = Math.min(m_red, Math.min(m_gre, m_blu));
        double diff = cmax - cmin;
        double h = -1;
        double s = -1;
                
        // if cmax and cmax are equal then h = 0
        if (cmax == cmin)
            h = 0;
 
        // if cmax equal r then compute h
        else if (cmax == m_red)
            h = (60 * ((m_gre - m_blu) / diff) + 360) % 360;
 
        // if cmax equal g then compute h
        else if (cmax == m_gre)
            h = (60 * ((m_blu - m_red) / diff) + 120) % 360;
 
        // if cmax equal b then compute h
        else if (cmax == m_blu)
            h = (60 * ((m_red - m_gre) / diff) + 240) % 360;
 
        // if cmax equal zero
        if (cmax == 0)
            s = 0;
        else
            s = (diff / cmax) * 100;
 
        // compute v
        double v = cmax * 100;

        m_hue = h;
        m_sat = s;
        m_val = v;
        
        //updates dashboard with RGB and HSV numbers
        SmartDashboard.putNumber("Red", m_red);
        SmartDashboard.putNumber("Blue", m_blu);
        SmartDashboard.putNumber("Green", m_gre);

        SmartDashboard.putNumber("Hue", m_hue);
        SmartDashboard.putNumber("Saturation", m_sat);
        SmartDashboard.putNumber("Value", m_val);

        SmartDashboard.putString("Color:", getColor());

    }

    @Override
    public void simulationPeriodic() {
        //this method will be called 1 per scheduler during sim
    }

    //accessors - 
    public double getHue(){
        return m_hue;
    }
    public double getSaturation(){
        return m_sat;
    }
    public double getValue(){
        return m_val;
    }

    //color output
    public String getColor(){
        //TODO: This will likely need to be updated with actual color values from the robot
        if( (m_hue > 200 && m_hue < 260) && (m_val > 50) && (m_sat > 75) ){
            return "RED";
        }
        //TODO: This will likely need to be updated with actual color values from the robot
        else if( (m_hue < 15 || m_hue > 350) && (m_val > 50) && (m_sat > 75) ){
            return "BLUE";
        }
        else {
            return "IDLE";
        }
    }
    public boolean shooterIsEmpty(){
        if(true){
            return true;
        }else{
            return false;
        }
    
    }
    public void advanceBall() {
        //advance the ball to the shooter
    }
}