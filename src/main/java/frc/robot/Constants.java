// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final class ClimberConstants {
    public static final int ClimbSolenoidRightRetract = 2;
    public static final int ClimbSolenoidRightExtend = 1;
    public static final int ClimbSolenoidRightChoke = 0;

    public static final int ClimbSolenoidLeftRetract = 5;
    public static final int ClimbSolenoidLeftExtend = 6;
    public static final int ClimbSolenoidLeftChoke = 7;

    public static final int AngleSolenoid = 3;
  }

  public static final class ShooterConstants {
    public static final int kLeftMotorCANId = 13;
    public static final int kRightMotorCANId = 14;

    public static final double kP = 6e-5; // 1.7637e-8;
    public static final double kI = 0;
    public static final double kIZone = 100; // rpm
    public static final double kD = 0.0000;
    public static final double kFF = 0.000183;
    public static final double kMaxVel = 5700;
    public static final double kMinVel = 0;
    public static final double kMaxAcc = 1500;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    // read more on this and change the value probably
    public static final double kAllowedError = 2;

    // Default angle for RPM calculations
    public static final double kDefaultYAngle = 12;

    // RPM tolerance for ramping
    public static final double kRPMTolerance = 50;
  }

  public static final class IntakeConstants {
    public static final int kCANRollerID = 7;
    public static final int kCANWinchID = 8;
    public static final int kCANInnerRollerID = 9;

    public static final double kRollerPower = 0.5;
    public static final double kWinchRaisingPower = 0.1;
    // Want to avoid the motor going faster than the spring / gravity can
    // lower the arm, leading to the cable going over the edge of the
    // spool
    public static final double kWinchLoweringPower = -0.2;
    public static final int kWinchMaxExpectedCurrent = 25;
    // Raised position is same as home
    public static final double kLiftRangeOfMotion = 8; // rotations
    public static final double kWinchRaisedPosition = 0;
    public static final double kWinchLoweredPosition = -4.5;
    public static final double kWinchPositionTolerance = 4;

    public static final int kIndexerLowerBottomBeltPort = 11;
    public static final int kIndexerUpperBottomBeltPort = 12;

    public static final int kIndexerTopBeltPort = 10;
  }

  public static final class IndexerConstants {
    // COLOR SENSOR
    public static final int kCargoMinProximity = 300;

    // Blue cargo HSV "boundaries"
    public static final double kMinBlueHue = 140;
    public static final double kMaxBlueHue = 185;
    public static final double kMinBlueSaturation = 40;
    public static final double kMaxBlueSaturation = 100;
    public static final double kMinBlueValue = 0.1;
    public static final double kMaxBlueValue = 15;

    // Red cargo HSV "boundaries"
    public static final double kMinRedHue = 15;
    public static final double kMaxRedHue = 50;
    public static final double kMinRedSaturation = 40;
    public static final double kMaxRedSaturation = 100;
    public static final double kMinRedValue = 0.7;
    public static final double kMaxRedValue = 10;
  }

  public static final class DriveConstants {
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(Math.PI * 6);
    public static final double kMotorRotationsPerWheelRotation = 7.56;

    // Spark MAX conversion factors
    public static final double kMetersPerMotorRotation =
        kWheelCircumferenceMeters / kMotorRotationsPerWheelRotation;
    public static final double kRPMToMetersPerSecond =
        kMetersPerMotorRotation / 60; // 60 seconds per minute

    // Motor controller ports (on 2022 bot)
    public static final int kLeftFront = 4;
    public static final int kLeftCenter = 5;
    public static final int kLeftBack = 6;

    public static final int kRightFront = 1;
    public static final int kRightCenter = 2;
    public static final int kRightBack = 3;

    // Speed/power limits during teleop
    public static final double kMaxForwardPower = 1.0;
    public static final double kMaxRotationPower = 0.6;

    // Split PID-related constants based on whether robot is turning/going straight
    public static final class StraightPID {
      // Basic PID constants
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;

      // Profiling
      public static final double kMaxVelocityMetersPerSecond = .1;
      public static final double kMaxAccelerationMeterPerSecondSquared = 5;

      // Feedforward
      public static final double kSVolts = 0.05; // Power!! for now
      public static final double kVVoltMetersPerSecond = 0.269 / 0.8856;

      // PID tolerances
      public static final double kDriveToleranceMeters = 0.1;
      public static final double kDriveVelocityToleranceMetersPerSecond = 0.2;
    }

    // Spark MAX Smart Motion constants
    // TODO: Tune these for the 2022 chassis
    public static final class SmartMotion {
      public static final double kP = 0;
      public static final double kTurnP = 8e-5;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kIz = 0;

      public static final double kFF = 0.000457;
      public static final double kTurnFF = 0.000657;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;
      public static final double kAllowedErr = 0.01; // meters

      // All of these are measured in RPM
      public static final double kMaxRPM = 5676;
      public static final double kMinVel = 0;
      public static final double kMaxVel = 2000;
      public static final double kMaxAcc = 1500;
    }

    public static final class TurnPID {
      public static final double kP = 0.006;
      public static final double kI = 0;
      public static final double kD = 0;

      // For use with encoder-based turning
      public static final double kMetersPerDegree = 0.0075;

      // Profiling
      public static final double kMaxVelocityDegreesPerSecond = 360 / 5;
      public static final double kMaxAccelerationDegreesPerSecondSquared = 240;

      // Feedforward
      private static final double kSecondsPerRotation = .85; // works for dead-er battery
      private static final double kTestPower = 0.4;

      public static final double kSVolts = 0.058;
      public static final double kVVoltDegreesPerSecond = 0.0008;
      // (kTestPower - kSVolts) / (356.494 / kSecondsPerRotation);

      public static final double kTurnToleranceDeg = 3.0;
      public static final double kTurnRateToleranceDegPerS = 20.0;
    }
  }

  public static final class ControllerConstants {
    // Controller ports
    public static final int kDriverPort = 0;
    public static final int kSecondaryPort = 1;

    // Thresholds for triggers & joysticks
    // TODO: Tune the trigger threshold
    public static final double kJoystickDeadband = 0.1;
    public static final double kTriggerActiveThreshold = 0.2;
  }
}
