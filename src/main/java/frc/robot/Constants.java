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
  public static final class DriveConstants {
    // Wheels are 8 inches in diameter, so their circumference can be calculated
    // (converted to meters for convenience)
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(Math.PI * 8);

    public static final double nominalVoltage = 12.0;

    // Motor controller ports (on 2019 bot)
    public static final int kLeftFront = 8;
    public static final int kLeftCenter = 7;
    public static final int kLeftBack = 6;

    public static final int kRightFront = 1;
    public static final int kRightCenter = 2;
    public static final int kRightBack = 3;

    // Split PID-related constants based on whether robot is turning/going straight
    public static final class StraightPID {
      // Basic PID constants
      public static final double kP = 1;
      public static final double kI = 0;
      public static final double kD = 0;

      // Profiling
      public static final double kMaxVelocityMetersPerSecond = 1.5;
      public static final double kMaxAccelerationMeterPerSecondSquared = 2;

      // Feedforward
      public static final double kSVolts = 0.131;
      public static final double kVVoltMetersPerSecond = 0.269 / 0.8856;
    }

    public static final class TurnPID {
      public static final double kP = 0.006;
      public static final double kI = 0;
      public static final double kD = 0;

      // Profiling
      public static final double kMaxVelocityDegreesPerSecond = 360 / 5;
      public static final double kMaxAccelerationDegreesPerSecondSquared = 120;

      // Feedforward
      private static final double kSecondsPerRotation = .85; // works for dead-er battery
      private static final double kTestPower = 0.4;

      public static final double kSVolts = 0.045;
      public static final double kVVoltDegreesPerSecond =
          (kTestPower - kSVolts) / (356.494 / kSecondsPerRotation);

      public static final double kTurnToleranceDeg = 3.0;
      public static final double kTurnRateToleranceDegPerS = 20.0;
    }
  }
}
