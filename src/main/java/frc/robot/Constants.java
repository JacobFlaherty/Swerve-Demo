// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double driveTrackWidth = 0.43;
    public static final double driveTrackLength = 0.73;

    public static final double rotateWheelGear = 1.0;
    public static final double velocityWheelGear = 1.0/8.16;//6.86
    public static final double wheelCircumfrence = Math.PI * 4*2.54 *0.01;

    public static final TrajectoryConfig config = new TrajectoryConfig(3, 2);


}
