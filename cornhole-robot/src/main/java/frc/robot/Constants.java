package frc.robot;

public final class Constants {
    private Constants(){throw new UnsupportedOperationException();}
    public static final double deadBandValue = 0.01;
    public static final double motorPowerMultiplier = 0.15;

    //
    // Constants for Launcher mechanism
    //
    public static final double LAUNCH_RESET_POWER  = -0.05;     // Power for returning arm to parked position
    public static final double LAUNCH_MAX_POSITION = 20500.0;
    public static final double LAUNCH_RPM          = 50.0;
    public static final double LAUNCH_VOLTAGE      = 11.5;      // Voltage compensation
    //
    public static final double LAUNCH_RAMP_RATE    = 0.1;       // Ramp time (in seconds) from zero power to full power
    //
    public static final double LAUNCH_P = 0.0;
    public static final double LAUNCH_I = 0.0;
    public static final double LAUNCH_D = 0.0;
    public static final double LAUNCH_F = 0.0;

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

    // Launcher CAN  bus IDs
    public static final int LAUNCH_CAN_ID = 25; // Set to correct value


}