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
  public final class ShooterConstants {
    public int LeftMotorCANId = 13;
    public int RightMotorCANId = 14;

    public double kvVolts = 0.000163;//0.002181818182; // Volts per RPM
    public double P = 0.0000;
    public double I = 0.0000001;
    public double D = 0.0000;

  }

  public final class IntakeConstants {
    public int kLiftLeftPort = 7;
    public int kLiftRightPort = 8;
    public int kRollerPort = 9;

    public int kIndexerLowerBottomBeltPort = 11;
    public int kIndexerUpperBottomBeltPort = 12;

    public int kIndexerTopBeltPort = 10;
  }

  public final class IndexerConstants {
    public int kIndexerLowerBottomBeltPort = 11;
    public int kIndexerUpperBottomBeltPort = 12;

    public int kIndexerTopBeltPort = 10;
  }


  public final class DriveConstants {
    // Wheels are 8 inches in diameter, so their circumference can be calculated
    // (converted to meters for convenience)
    public double kWheelCircumferenceMeters = Units.inchesToMeters(Math.PI * 7.736);
    public double kMotorRotationsPerWheelRotation = 7.56 * 2.8;

    public double nominalVoltage = 12.0;

    // Motor controller ports (on 2022 bot)
    public int kLeftFront = 4;
    public int kLeftCenter = 5;
    public int kLeftBack = 6;

    public int kRightFront = 1;
    public int kRightCenter = 2;
    public int kRightBack = 3;

    // Split PID-related constants based on whether robot is turning/going straight
    public final class StraightPID {
      // Basic PID constants
      public double kP = 0;
      public double kI = 0;
      public double kD = 0;

      // Profiling
      public double kMaxVelocityMetersPerSecond = .1;
      public double kMaxAccelerationMeterPerSecondSquared = 5;

      // Feedforward
      public double kSVolts = 0.05; // Power!! for now
      public double kVVoltMetersPerSecond = 0.269 / 0.8856;

      // PID tolerances
      public double kDriveToleranceMeters = 0.1;
      public double kDriveVelocityToleranceMetersPerSecond = 0.2;
    }

    public final class TurnPID {
      public double kP = 0.006;
      public double kI = 0;
      public double kD = 0;

      // Profiling
      public double kMaxVelocityDegreesPerSecond = 360 / 5;
      public double kMaxAccelerationDegreesPerSecondSquared = 120;

      // Feedforward
      private double kSecondsPerRotation = .85; // works for dead-er battery
      private double kTestPower = 0.4;

      public double kSVolts = 0.045;
      public double kVVoltDegreesPerSecond =
          (kTestPower - kSVolts) / (356.494 / kSecondsPerRotation);

      public double kTurnToleranceDeg = 3.0;
      public double kTurnRateToleranceDegPerS = 20.0;
    }
  }
}
/* feedforward feedback gains
// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.16741;
    public static final double kvVoltSecondsPerMeter = .12549;
    public static final double kaVoltSecondsSquaredPerMeter = 0.20605;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = .16767;
//differential drive kinematics
public static final double kTrackwidthMeters = 8.3588;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

//max trajectory vel/accel
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

//ramsete patterns.
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
*/
