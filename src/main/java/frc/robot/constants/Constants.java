// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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
  public static final Mode currentMode = Mode.SIM;

  public enum Mode {
    COMP,
    SIM,
    TEST
  }

  public static final class OperatorConstants {
    public static final double kDEADBAND_LEFT_Y = 0.1;
    public static final double kDEADBAND_LEFT_X = 0.1;

    public static final int kOperatorControllerPort = 2;
    public static final int kDriverControllerPort = 3;
  }

  public static final class PivotConstants {
    public static final double kPIVOT_MOTOR_TO_OUTPUT_SHAFT_RATIO = 10;
    public static final double kJKG_METERS_SQUARED = 0.1;
    public static final double kPIVOT_LENGTH_METERS = 0.222;
    public static final double kMIN_ANGLE_RAD = -46;
    public static final double kMAX_ANGLE_RAD = 125;
    public static final double kSTARTING_ANGLE_RAD = Math.toRadians(0);
  }
}
