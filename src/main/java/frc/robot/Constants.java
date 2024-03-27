package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Storage for robot-wide constants
 */
public final class Constants {
    private Constants(){} // block instantiation

    public static final int           NEO_COUNTS_PER_REVOLUTION   = 42;
    public static final int           NEO_CURRENT_LIMIT_AMPS      = 50;
    public static final double        NEO_MAX_VOLTAGE             = 12;
 
    public static final int           FRONT_LEFT_DRIVE_MOTOR_ID   = 1;
    public static final int           FRONT_RIGHT_DRIVE_MOTOR_ID  = 2;
    public static final int           BACK_LEFT_DRIVE_MOTOR_ID    = 3;
    public static final int           BACK_RIGHT_DRIVE_MOTOR_ID   = 4;
 
    public static final int           ARM_LEFT_MOTOR_ID           = 5;
    public static final int           ARM_RIGHT_MOTOR_ID          = 6;
 
    public static final int           SHOOTER_TOP_MOTOR_ID        = 7;
    public static final int           SHOOTER_BOTTOM_MOTOR_ID     = 8;
 
    public static final int           INTAKE_MOTOR_ID             = 9;

    public static final int           HANGER_LEFT_MOTOR_ID        = 10;
    public static final int           HANGER_RIGHT_MOTOR_ID       = 11;
 
    public static final double        LOOP_TIME_MS                = 20;
    public static final double        LOOP_TIME_SECONDS           = LOOP_TIME_MS / 1000;


    public static final String        LIMELIGHT_NAME              = "limelight";
    public static final Translation2d BLUE_SPEAKER                = new Translation2d(-8.308975, 1.442593);
    public static final Translation2d RED_SPEAKER                 = new Translation2d(8.308975, 1.442593);
           
    public static final double        SLOW_MODE_SPEED             = 0.5;

    public static final boolean       INCLUDE_AUTO                = true;
    public static final boolean       INCLUDE_LIMELIGHT           = true;
}
