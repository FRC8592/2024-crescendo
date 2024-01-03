package frc.robot;

public class Constants {
    public Constants() {
        throw new UnsupportedOperationException("The Constants class can't be instantiated!");
    }

    /****************************************************************************************
     * Conversions
     *****************************************************************************************/
    public static final double CONVERSION_METERS_SECOND_TO_TICKS = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));
    public static final double CONVERSION_RPM_TO_TICKS_100_MS = 2048.0 / 600.0;

    /****************************************************************************************
     * Controllers
     *****************************************************************************************/
    public static final double CONTROLLER_JOYSTICK_DEADBAND = 0.01;
    public static final int CONTROLLER_DRIVER_PORT = -1; // TODO: Value isn't set yet. Usually 0.
    public static final int CONTROLLER_OPERATOR_PORT = -1; // TODO: Value isn't set yet. Usually 1.


    /****************************************************************************************
     * Swerve (Drivetrain)
     *
     * TEAL is front-left
     * ORANGE is front-right
     * BLACK is back-right
     * WHITE is back-left
     *****************************************************************************************/
    public static final int SWERVE_FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID = -1; // TODO: Value isn't set yet.
    public static final int SWERVE_FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID = -1; // TODO: Value isn't set yet.
    public static final int SWERVE_FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID = -1; // TODO: Value isn't set yet.

    public static final int SWERVE_FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID = -1; // TODO: Value isn't set yet.
    public static final int SWERVE_FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID = -1; // TODO: Value isn't set yet.
    public static final int SWERVE_FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID = -1; // TODO: Value isn't set yet.

    public static final int SWERVE_BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID = -1; // TODO: Value isn't set yet.
    public static final int SWERVE_BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID = -1; // TODO: Value isn't set yet.
    public static final int SWERVE_BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID = -1; // TODO: Value isn't set yet.

    public static final int SWERVE_BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID = -1; // TODO: Value isn't set yet.
    public static final int SWERVE_BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID = -1; // TODO: Value isn't set yet.
    public static final int SWERVE_BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID = -1; // TODO: Value isn't set yet.

    public static final int SWERVE_PIGEON_CAN_ID = -1; // TODO: Value isn't set yet.

    public static final double SWERVE_THROTTLE_kP = 0.02;
    public static final double SWERVE_THROTTLE_kI = 0.0;
    public static final double SWERVE_THROTTLE_kD = 0.01;

    public static final double SWERVE_STEER_kP = 0.2;
    public static final double SWERVE_STEER_kI = 0.0;
    public static final double SWERVE_STEER_kD = 0.1;

    public static final double SWERVE_DRIVE_TRAIN_WIDTH = -1; // meters // TODO: Value isn't set yet.
    public static final double SWERVE_DRIVE_TRAIN_LENGTH = -1; // meters // TODO: Value isn't set yet.
    public static final double SWERVE_WHEEL_CIRCUMFERENCE = 4 * Math.PI; // TODO: Check that this is accurate

    public static final double SWERVE_FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-1);
    public static final double SWERVE_FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-1); // TODO: Value isn't set yet.
    public static final double SWERVE_BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-1);
    public static final double SWERVE_BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-1);

    public static final double SWERVE_TRANSLATE_POWER_FAST = 1.0; // Scaling for teleop driving. 1.0 is maximum
    public static final double SWERVE_ROTATE_POWER_FAST = 0.75; // Scaling for teleop driving. 1.0 is maximum
    public static final double SWERVE_TRANSLATE_POWER_SLOW = 0.15; // Scaling for teleop driving. 1.0 is maximum
    public static final double SWERVE_ROTATE_POWER_SLOW = 0.15; // Scaling for teleop driving. 1.0 is maximum

    public static final double SWERVE_MAX_VOLTAGE = 12.0;
    public static final double TELEOP_CURRENT_LIMIT = -1; // TODO: Value isn't set yet.
    public static final double SWERVE_MAX_VELOCITY_METERS_PER_SECOND = 4.5;
}
