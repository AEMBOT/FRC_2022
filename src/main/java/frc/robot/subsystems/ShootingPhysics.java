package frc.robot.subsystems;

import javax.swing.ListModel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShootingPhysics {
    //all in metric system 
    private double distance;
    private double maxHeiht; 
    private int shooterAngle = 24;
    private double shooterHeight = 0.3048;
    private double gravity = 9.8;
    private int flywheelRadius = 4;
    private double targetHeight = 2.7432;

    public ShootingPhysics(){}


    public double RPMtoAngularVelocity(int rpm){
        double omega = (2 * Math.PI * rpm) / 60;
        return omega;
    }
    public double angularToLinearVelocity(double angularVelocity){
        //two flywheels 
        double velocity = RPMtoAngularVelocity(4000) * flywheelRadius; 
        return velocity; 
    }

    public double linearToRPM(double v){
        double angular = v / flywheelRadius;
        double rpm = (angular * 60) / (2 * Math.PI);
        return rpm;
    }

    public double ballMaxHeight(double dist, double initalVelocity, int launchAngle, double launchHeight, double gravity){
        double maxHeight = ((dist * Math.tan(launchAngle)) + (0.5 * gravity * Math.pow(Math.cos(launchAngle),2)) + launchHeight);
        return maxHeight;
    }

    public double RPMforDistanceX(double dist, double launchAngle){
        //2 flywheels, rpm / 2 
        //dist is limelighttargeting.getdistance()
        double neededVelocity = Math.sqrt((dist * 9.8) / (Math.sin(2 * launchAngle)));
        double RPM = linearToRPM(neededVelocity);
        return RPM;
    }
}
