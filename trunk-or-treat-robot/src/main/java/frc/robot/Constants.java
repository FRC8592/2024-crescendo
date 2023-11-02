package frc.robot;

public final class Constants {
    private Constants(){throw new UnsupportedOperationException();}
    public static final double deadBandValue = 0.01;
    public static final double motorPowerMultiplier = 0.3;
    public static final double LAUNCH_DOWN_POWER = -0.1;
    public static final double LAUNCH_MAX_POSITION = 4500.0;
    public static final double DOWN_MIN_POSITION = 1500;
    // public static final double LAUNCH_MOTOR_kP = 0.20;
    // public static final double LAUNCH_MOTOR_kI = 0;
    // public static final double LAUNCH_MOTOR_kD = 0;
    // public static final double LAUNCH_MOTOR_kF = 0.25;
    // public static final double LAUNCH_MOTOR_kP_BRAKE = 0.05;
    // public static final double LAUNCH_MOTOR_kI_BRAKE = 0;
    // public static final double LAUNCH_MOTOR_kD_BRAKE = 0;
    // public static final double LAUNCH_MOTOR_kF_BRAKE = -0.2;
    public static final int TICKS_UP_POSITION = 6000;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware ID Mapping
/////////////////////////////////////////////////////////////////////////////////////////////////////

    // Joystick USB IDs
    public static final int DRIVER_GAMEPAD_PORT   = 0;

    // Drivetrain CAN bus IDs
    public static final int LEFT_FRONT_CAN_ID  = 21;
    public static final int LEFT_BACK_CAN_ID   = 3;
    public static final int RIGHT_FRONT_CAN_ID = 4;
    public static final int RIGHT_BACK_CAN_ID  = 2;

    // Launcher CAN  bus IDs
    public static final int LAUNCH_CAN_ID = 30;
    public static final int LAUNCH_2_CAN_ID = 26;


}