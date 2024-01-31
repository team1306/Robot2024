package frc.robot;

/**
 * Storage for robot-wide constants
 */
public final class Constants {
    private Constants(){} // block instantiation

    public static final int    NEO_COUNTS_PER_REVOLUTION   = 42;
    public static final int    NEO_CURRENT_LIMIT_AMPS      = 50;
    public static final double NEO_MAX_VOLTAGE             = 12;

    public static final int    FRONT_LEFT_DRIVE_MOTOR_ID   = 0;
    public static final int    FRONT_RIGHT_DRIVE_MOTOR_ID  = 1;
    public static final int    BACK_LEFT_DRIVE_MOTOR_ID    = 2;
    public static final int    BACK_RIGHT_DRIVE_MOTOR_ID   = 3;

    public static final int    ARM_LEFT_MOTOR_ID           = 4;
    public static final int    ARM_RIGHT_MOTOR_ID          = 5;

    public static final int    SHOOTER_TOP_MOTOR_ID        = 6;
    public static final int    SHOOTER_BOTTOM_MOTOR_ID     = 7;

    public static final int    INTAKE_MOTOR_ID             = 8;

    public static final int    LOOP_TIME_MS                = 20;
    public static final int    LOOP_TIME_SECONDS            = LOOP_TIME_MS / 1000;

    
    public static final String LIMELIGHT_NAME              = "limelight";

}
