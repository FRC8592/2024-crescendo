package frc.robot;

public final class Constants {
    public final class SHARED {
        public static final String LOG_FOLDER = "CustomLogs";
    }

    public final class CONVERSIONS {
        public static final double METERS_SECOND_TO_TICKS = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));
        public static final double RPM_TO_TICKS_100_MS = 2048.0 / 600.0;
        public static final double DEG_TO_RAD = 0.0174533;
        public static final double IN_TO_METERS = 0.0254;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public final class NOTELOCK {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/NoteLock/";
        public static final String LIMELIGHT_NAME = ""; // TODO: Not set yet
        public static final double LOCK_ERROR = -1; // TODO: Not set yet
        public static final double CLOSE_ERROR = -1; // TODO: Not set yet
        public static final double CAMERA_HEIGHT = -1; // TODO: Not set yet
        public static final double kP = -1; // TODO: Not set yet
        public static final double kI = -1; // TODO: Not set yet
        public static final double kD = -1; // TODO: Not set yet
    }

    public final class INTAKE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";
        public static final int TOP_MOTOR_CAN_ID = 35;
        public static final int BOTTOM_MOTOR_CAN_ID = 34;
        public static final double TOP_MOTOR_kP = 0.0001;
        public static final double TOP_MOTOR_kI = 0.000;
        public static final double TOP_MOTOR_kD = 0.000;
        public static final double TOP_MOTOR_kF = 0.000;
        public static final double BOTTOM_MOTOR_kP = 0.0001;
        public static final double BOTTOM_MOTOR_kI = 0.000;
        public static final double BOTTOM_MOTOR_kD = 0.000;
        public static final double BOTTOM_MOTOR_kF = 0.000;
        public static final double MINIMUM_ROLLER_SPEED = 0;
        public static final double ROBOT_SPEED_MULTIPLIER = 100;
        public static final double SPEED_TOP = 1000;
        public static final double SPEED_BOTTOM = 1000;
    }

    public final class SHOOTER {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Shooter/";
    }

    public final class ELEVATOR {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Elevator/";
    }

    public final class POWER {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Power/";
        public static final int PDH_CAN_ID = 1;
        public static final int VOLTAGE_SMOOTHING_LENGTH = 50; // TODO: Arbitrary (needs testing)
        public static final double DISABLED_LOW_BATTERY_VOLTAGE = 11.5; // TODO: Arbitrary (needs testing)
        public static final double TELEOP_LOW_BATTERY_VOLTAGE = 10.5; // TODO: Arbitrary (needs testing)
    }

    public final class LEDS {
        public static final int LED_LENGTH = 8;
        public static final double MINIMUM_VOLTAGE = 9.0;
        public static final int PULSE_METHOD_SPEED = 5;
        public static final int PULSE_SIZE = 2;
        public static final int PULSE_GAP = 5;
    }

    public final class SWERVE {
       /****************************************************************************************
        * BLACK is front-left
        * ORANGE is front-right
        * TEAL is back-right
        * WHITE is back-left
        *****************************************************************************************/
        public static final int BLACK_FRONT_LEFT_DRIVE_CAN = 14; 
        public static final int BLACK_FRONT_LEFT_STEER_CAN = 13; 
        public static final int BLACK_FRONT_LEFT_ENCODER_CAN = 15; 

        public static final int ORANGE_FRONT_RIGHT_DRIVE_CAN = 20;
        public static final int ORANGE_FRONT_RIGHT_STEER_CAN = 19;
        public static final int ORANGE_FRONT_RIGHT_ENCODER_CAN = 21;

        public static final int TEAL_BACK_LEFT_DRIVE_CAN = 11;
        public static final int TEAL_BACK_LEFT_STEER_CAN = 10;
        public static final int TEAL_BACK_LEFT_ENCODER_CAN = 12;

        public static final int WHITE_BACK_RIGHT_DRIVE_CAN = 17;
        public static final int WHITE_BACK_RIGHT_STEER_CAN = 16;
        public static final int WHITE_BACK_RIGHT_ENCODER_CAN = 18;

        public static final double THROTTLE_kP = 0.02;
        public static final double THROTTLE_kI = 0.0;
        public static final double THROTTLE_kD = 0.01;

        public static final double STEER_kP = 0.2;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.1;

        public static final double DRIVE_TRAIN_WIDTH = 0.527; // meters
        public static final double DRIVE_TRAIN_LENGTH = 0.478; // meters 
        public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;

        public static final double BLACK_FRONT_LEFT_STEER_OFFSET = -Math.toRadians(145.898); // 354.639
        public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(343.213); // 0
        public static final double TEAL_BACK_LEFT_STEER_OFFSET = -Math.toRadians(67.500);    // 69.434
        public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -Math.toRadians(355.166); // 6.768

        public static final double TRANSLATE_POWER_FAST = 1.0; // Scaling for teleop driving. 1.0 is maximum
        public static final double ROTATE_POWER_FAST = 0.75; // Scaling for teleop driving. 1.0 is maximum
        public static final double TRANSLATE_POWER_SLOW = 0.15; // Scaling for teleop driving. 1.0 is maximum
        public static final double ROTATE_POWER_SLOW = 0.15; // Scaling for teleop driving. 1.0 is maximum

        public static final double MAX_VOLTAGE = 12.0;
        public static final double TELEOP_CURRENT_LIMIT = 15.0; 
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
    }

    public final class PIGEON {
        public static final int CAN_ID = 17;
    }
    public class ROBOT {
        public static final double JOYSTICK_DEADBAND = 0.01;
        private static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
    }
}
