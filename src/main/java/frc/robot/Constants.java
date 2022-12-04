// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // The default period of the robot loop is 20 milliseconds (0.02 seconds)
  public static final double kRobotLoopPeriod = TimedRobot.kDefaultPeriod;

  /** Solenoid ports on the PCM for use in the climber subsystem. */
  public static final class ClimberConstants {
    public static final int kClimbSolenoidRightRetract = 2;
    public static final int kClimbSolenoidRightExtend = 1;
    public static final int kClimbSolenoidRightChoke = 0;

    public static final int kClimbSolenoidLeftRetract = 6;
    public static final int kClimbSolenoidLeftExtend = 7;
    public static final int kClimbSolenoidLeftChoke = 4;

    public static final int kAngleSolenoid = 3;
  }

  /** Shooter-related constants, including CAN IDs and PIDF coefficients. */
  public static final class ShooterConstants {
    public static final int kLeftMotorCANId = 13;
    public static final int kRightMotorCANId = 14;

    // PIDF constants for velocity closed loop control
    public static final double kP = 6e-5;
    public static final double kI = 0;
    public static final double kIZone = 0;
    public static final double kD = 0.0000;
    public static final double kFF = 0.000183;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;

    // Default Limelight Y angle for RPM calculations
    public static final double kDefaultYAngle = 12;

    // RPM tolerance for ramping
    public static final double kRPMTolerance = 50;
  }

  /**
   * Intake-related constants, including motors powers, current limits, & the intake lift range of
   * motion.
   */
  public static final class IntakeConstants {
    // CAN IDs for intake-related motors
    public static final int kCANRollerID = 7;
    public static final int kCANLiftID = 8;

    // Powers for moving various parts of the intake
    public static final double kRollerPower = 0.5;
    public static final double kLiftRaisingPower = 0.1;
    public static final double kLiftLoweringPower = -0.1;

    // Current limit for when the intake is fully up
    public static final int kLiftMaxExpectedCurrent = 35;

    public static final double kLiftRangeOfMotion = 8; // rotations
  }

  /** Indexer motor CAN IDs as well as color sensor tolerances for detecting cargo. */
  public static final class IndexerConstants {
    // Motor CAN IDs
    public static final int kCANIndexerUpperBottomBeltID = 12;
    public static final int kCANIndexerUpperTopBeltID = 10;
    public static final int kCANIndexerLowerBottomBeltID = 11;
    public static final int kCANIndexerLowerTopBeltID = 9;

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

  /**
   * Drivetrain-related constants, including several subclasses for PID-controlled/motion-profiled
   * motion.
   */
  public static final class DrivetrainConstants {
    // Drivetrain measurements
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(Math.PI * 6);

    // This is the same as the gear ratio for the drive motor gearboxes
    public static final double kMotorRotationsPerWheelRotation = 7.56;

    // Encoder conversions
    public static final double kMetersPerMotorRotation =
        kWheelCircumferenceMeters / kMotorRotationsPerWheelRotation;
    public static final double kRPMToMetersPerSecond =
        kMetersPerMotorRotation / 60; // 60 seconds per minute

    // Track width (found using sysid)
    public static final double kEffectiveTrackWidth = 0.69883;

    // Ramsete constants
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

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

    // Max velocity & acceleration during automonous driving
    public static final double kMaxVelocityMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;

    // Feedforward constants obtained from SysId
    public static final double kSLinear = 0.17129; // V
    public static final double kVLinear = 2.0522; // V*s/m
    public static final double kALinear = 0.4259; // V*s^2/m

    public static final double kSAngular = 0.36201; // V
    public static final double kVAngular = 2.1981; // V*s/m
    public static final double kAAngular = 0.20213; // V*s^2/m

    // Feedback (PID) constants for driving straight using the Spark Max onboard control
    public static final double kLinearP = 0.066676;
    public static final double kLinearI = 0;
    public static final double kLinearD = 0;

    // Feedforward is done on the rio, not the Spark Maxes
    public static final double kLinearFF = 0;

    // Minimum/maximum output for onboard Spark Max control
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;

    /** PID/motion profiling constants for turning in place (used in TurnDegrees). */
    // TODO: Transition these to voltages instead of power
    public static final class TurnPID {
      public static final double kP = 0.006;
      public static final double kI = 0;
      public static final double kD = 0;

      // Profiling
      public static final double kMaxVelocityDegreesPerSecond = 360 / 5;
      public static final double kMaxAccelerationDegreesPerSecondSquared = 240;

      // Feedforward
      public static final double kS = 0.058;
      public static final double kVSecondsPerDegree = 0.0008;
    }
  }

  /** Controller ports & deadbands. */
  public static final class ControllerConstants {
    // Controller ports
    public static final int kDriverPort = 0;
    public static final int kSecondaryPort = 1;

    // Thresholds for triggers & joysticks
    public static final double kJoystickDeadband = 0.1;
    public static final double kTriggerActiveThreshold = 0.1;
  }
}
