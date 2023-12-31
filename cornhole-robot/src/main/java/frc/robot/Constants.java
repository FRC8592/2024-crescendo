package frc.robot;

public final class Constants {
    private Constants(){throw new UnsupportedOperationException();}
    public static final double deadBandValue = 0.01;
    public static final double motorPowerMultiplier = 0.3;

    // Constants for turret/shooter
    // public static final double TURRET_LAUNCH_SPEED = 0.5;
    // public static final int TURRET_ROTATION_LIMIT = 1700;
    // public static final double TICKS_PER_DEGREE = 1700/90; //1700 ticks for about 90 degrees. Refine later with CAD files.
    // public static final double TURN_TO_RANGE = 20;
    // public static final double FLYWHEEL_MULTIPLIER = 0.2;

    // public static final double FOUR_BAR_kP = 0.0;
    // public static final double FOUR_BAR_kI = 0.0;
    // public static final double FOUR_BAR_kD = 0.0;

    // public static final double FOURBAR_LAUNCH_MAX_ROTATION_TICKS = 1024.0;

    //
    // Constants for Launcher mechanism
    //
    //public static final double LAUNCH_ACCEL_POWER =  0.8;
    public static final double LAUNCH_VELOCTY_TICKS_100MS = 5000;
    public static final double LAUNCH_DOWN_POWER = -0.05;
    public static final double LAUNCH_MAX_POSITION = 7500.0;
    public static final double DOWN_MIN_POSITION = 1000;
    public static final double LAUNCH_MOTOR_kP = 0.20;
    public static final double LAUNCH_MOTOR_kI = 0;
    public static final double LAUNCH_MOTOR_kD = 0;
    public static final double LAUNCH_MOTOR_kF = 0.25;
    public static final double LAUNCH_MOTOR_kP_BRAKE = 0.05;
    public static final double LAUNCH_MOTOR_kI_BRAKE = 0;
    public static final double LAUNCH_MOTOR_kD_BRAKE = 0;
    public static final double LAUNCH_MOTOR_kF_BRAKE = -0.2;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware ID Mapping
/////////////////////////////////////////////////////////////////////////////////////////////////////

    // Joystick USB IDs
    public static final int DRIVER_GAMEPAD_PORT   = 0;
    public static final int OPERATOR_GAMEPAD_PORT = 1;

    // Drivetrain CAN bus IDs
    public static final int LEFT_FRONT_CAN_ID  = 21;
    public static final int LEFT_BACK_CAN_ID   = 3;
    public static final int RIGHT_FRONT_CAN_ID = 4;
    public static final int RIGHT_BACK_CAN_ID  = 2;

    // Turret CAN bus IDs
    public static final int TURRET_ROTATE_CAN_ID = 12;  // Control rotation of turret
    public static final int TURRET_LAUNCH_CAN_ID = 25;   // High speed flywheel

    // Launcher CAN  bus IDs
    public static final int LAUNCH_CAN_ID = 30; // Set to correct value
    public static final int LAUNCH_2_CAN_ID = 26;


}