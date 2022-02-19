// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        // Motor controller ports (on 2019 bot)
        public static final int kLeftFront = 8;
        public static final int kLeftCenter = 7;
        public static final int kLeftBack = 6;
        
        public static final int kRightFront = 1;
        public static final int kRightCenter = 2;
        public static final int kRightBack = 3;
    }
}
