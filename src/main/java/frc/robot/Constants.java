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
  /** Solenoid ports on the PCM */
  public final class ClimberConstants {
    public static final int kClimbSolenoidRightRetract = 2;
    public static final int kClimbSolenoidRightExtend = 1;
    public static final int kClimbSolenoidRightChoke = 0;

    public static final int kClimbSolenoidLeftRetract = 6;
    public static final int kClimbSolenoidLeftExtend = 7;
    public static final int kClimbSolenoidLeftChoke = 4;

    public static final int kAngleSolenoid = 3;
  }

  public static final class ShooterConstants {
    public static final int kLeftMotorCANId = 13;
    public static final int kRightMotorCANId = 14;

    public static final double kP = 6e-5; // 1.7637e-8;
    public static final double kI = 0;
    public static final double kIZone = 0;
    public static final double kD = 0.0000;
    public static final double kFF = 0.000183;
    public static final double kMaxVel = 5700;
    public static final double kMinVel = 0;
    public static final double kMaxAcc = 1500;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;

    // Default Limelight Y angle for RPM calculations
    public static final double kDefaultYAngle = 12;

    // RPM tolerance for ramping
    public static final double kRPMTolerance = 50;
  }

  public static final class IntakeConstants {
    // CAN IDs for intake-related motors
    public static final int kCANRollerID = 7;
    public static final int kCANLiftID = 8;
    public static final int kCANInnerRollerID = 9;
    public static final int kCANIndexerLowerBottomBeltID = 11;

    // Powers for moving various parts of the intake
    public static final double kRollerPower = 0.5;
    public static final double kLiftRaisingPower = 0.1;
    public static final double kLiftLoweringPower = -0.2;

    // Current limit for when the intake is fully up
    public static final int kLiftMaxExpectedCurrent = 30;

    public static final double kLiftRangeOfMotion = 8; // rotations
  }

  public static final class IndexerConstants {
    // Motor CAN IDs
    public static final int kCANIndexerUpperBottomBeltID = 12;
    public static final int kCANIndexerTopBeltID = 10;

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
    // Measurements for Spark Max conversion factors
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(Math.PI * 6);
    public static final double kMotorRotationsPerWheelRotation = 7.56;

    public static final double kMetersPerMotorRotation =
        kWheelCircumferenceMeters / kMotorRotationsPerWheelRotation;
    public static final double kRPMToMetersPerSecond =
        kMetersPerMotorRotation / 60; // 60 seconds per minute

    // Motor controller ports (on Tupperware)
    // FIXME: Front/back IDs might be swapped (not that it affects anything)
    public static final int kLeftFront = 1;
    public static final int kLeftCenter = 2;
    public static final int kLeftBack = 3;

    public static final int kRightFront = 4;
    public static final int kRightCenter = 5;
    public static final int kRightBack = 6;

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

    public static final class TurnPID {
      public static final double kP = 0.006;
      public static final double kI = 0;
      public static final double kD = 0;

      // For use with encoder-based turning
      public static final double kMetersPerDegree = 0.0075;

      // Profiling
      public static final double kMaxVelocityDegreesPerSecond = 360 / 5;
      public static final double kMaxAccelerationDegreesPerSecondSquared = 240;

      // Feedforward (both in power units, i.e. on [-1, 1])
      public static final double kS = 0.058;
      public static final double kVDegreesPerSecond = 0.0008;

      public static final double kTurnToleranceDeg = 3.0;
      public static final double kTurnRateToleranceDegPerS = 20.0;
    }

    // Spark MAX Smart Motion constants
    public static final class SmartMotion {
      // PIDF constants
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kIz = 0;
      public static final double kFF = 0.000457;

      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;
      public static final double kAllowedErr = 0.01; // meters

      // All of these are measured in RPM
      public static final double kMaxRPM = 5676;
      public static final double kMinVel = 0;
      public static final double kMaxVel = 2000;
      public static final double kMaxAcc = 1500;
    }
  }

  public static final class ControllerConstants {
    // Controller ports
    public static final int kDriverPort = 0;
    public static final int kSecondaryPort = 1;

    // Thresholds for triggers & joysticks
    public static final double kJoystickDeadband = 0.1;
    public static final double kTriggerActiveThreshold = 0.1;
  }
}
