// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class IntakeConstants {
    public static final int ROLLER_Kraken_ID = 9;
    public static final int ARM_Neo_ID = 20;
    public static final int ARM_ENCODER_PWM_ID = 1;
  }

  public static class IndexerConstants {
    public static final int Neo_1_ID = 15;
    public static final int Neo_2_ID = 16;
    public static final int Neo_3_ID = 17;
  }

  public static class TurretConstants {
    public static final int Motor_Kraken_ID = 10;
    public static final int Encoder_PWM_ID = 2;
    public static final double MOTOR_TO_TURRET_RATIO = 48.0;
    public static final double ENCODER_TO_TURRET_RATIO = 10.0;

    public static final double FORWARD_LIMIT = 180;
    public static final double REVERSE_LIMIT = -180;

    public static final double MAX_MANUAL_PERCENT = 0.35;

    public static final double CRUISE_VELOCITY = 700;
    public static final double ACCELERATION = 1400;
    public static final double JERK = 5000;

    public static final double kP = 12;
    public static final double kI = 0;
    public static final double kD = 0.2;

    public static final double SCAN_SPEED = 1.5;

    public static final double SHOT_LEAD_TIME = 0.35;

    public static final double CAMERA_OFFSET_X = 0;
    public static final double CAMERA_OFFSET_Y = -0.15;
    public static final double CAMERA_OFFSET_Z = 0.46;

    public static final Translation3d BLUE_HUB_POSITION = new Translation3d(4.597, 4.035, 1.575);
    public static final Translation3d RED_HUB_POSITION = new Translation3d(11.938, 4.035, 1.575);
  }

  public static class ShooterConstants {
    public static final int FLYWHEEL_1_Kraken_ID = 11;
    public static final int FLYWHEEL_2_Kraken_ID = 12;
    public static final int HOOD_PWM_ID = 3;
  }
}
