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
        public static final double METERS_TO_FEET = 3.28084;
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
        public static final int TOP_MOTOR_CAN_ID = 15;
        public static final int BOTTOM_MOTOR_CAN_ID = 18;
        public static final double TOP_MOTOR_kP = 0.01;
        public static final double TOP_MOTOR_kI = 0.001;
        public static final double TOP_MOTOR_kD = 0.001;
        public static final double BOTTOM_MOTOR_kP = 0.01;
        public static final double BOTTOM_MOTOR_kI = 0.001;
        public static final double BOTTOM_MOTOR_kD = 0.001;
        public static final double MINIMUM_ROLLER_SPEED = 0;
        public static final double ROBOT_SPEED_MULTIPLIER = 100;
    }

    public final class SHOOTER {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Shooter/";

        public static final int TOP_SHOOTER_MOTOR_CAN_ID = 29;
        public static final int BOTTOM_SHOOTER_MOTOR_CAN_ID = 28;
        public static final int FEEDER_MOTOR_CAN_ID = 31;

        //Change right motor to inverted using documentation
        public static final double TOP_SHOOTER_MOTOR_SPEED = 0.25;
        public static final double BOTTOM_SHOOTER_MOTOR_SPEED = TOP_SHOOTER_MOTOR_SPEED;
        public static final double TOP_SHOOTER_MOTOR_kP = 0.00007; //PID
        public static final double TOP_SHOOTER_MOTOR_kI = 0.0000005; //PID
        public static final double TOP_SHOOTER_MOTOR_kD = 0.0000035; //PID
        public static final double BOTTOM_SHOOTER_MOTOR_kP = 0.00007; //PID
        public static final double BOTTOM_SHOOTER_MOTOR_kI = 0.0000005;//PID
        public static final double BOTTOM_SHOOTER_MOTOR_kD = 0.0000035; //PID

        public static final int ACCEPTABLE_RANGE = 50;
        //Lookup table
        public static final double[][] RANGE_TABLE = {
            {0,3000}
        };
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
        * TEAL is front-left
        * ORANGE is front-right
        * BLACK is back-right
        * WHITE is back-left
        *****************************************************************************************/
        public static final int TEAL_FRONT_LEFT_DRIVE_CAN = -1; // TODO: Value isn't set.
        public static final int TEAL_FRONT_LEFT_STEER_CAN = -1; // TODO: Value isn't set.
        public static final int TEAL_FRONT_LEFT_ENCODER_CAN = -1; // TODO: Value isn't set.

        public static final int ORANGE_FRONT_RIGHT_DRIVE_CAN = -1; // TODO: Value isn't set.
        public static final int ORANGE_FRONT_RIGHT_STEER_CAN = -1; // TODO: Value isn't set.
        public static final int ORANGE_FRONT_RIGHT_ENCODER_CAN = -1; // TODO: Value isn't set.

        public static final int BLACK_BACK_LEFT_DRIVE_CAN = -1; // TODO: Value isn't set.
        public static final int BLACK_BACK_LEFT_STEER_CAN = -1; // TODO: Value isn't set.
        public static final int BLACK_BACK_LEFT_ENCODER_CAN = -1; // TODO: Value isn't set.

        public static final int WHITE_BACK_RIGHT_DRIVE_CAN = -1; // TODO: Value isn't set.
        public static final int WHITE_BACK_RIGHT_STEER_CAN = -1; // TODO: Value isn't set.
        public static final int WHITE_BACK_RIGHT_ENCODER_CAN = -1; // TODO: Value isn't set.

        public static final double THROTTLE_kP = 0.02;
        public static final double THROTTLE_kI = 0.0;
        public static final double THROTTLE_kD = 0.01;

        public static final double STEER_kP = 0.2;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.1;

        public static final double DRIVE_TRAIN_WIDTH = -1; // meters // TODO: Value isn't set.
        public static final double DRIVE_TRAIN_LENGTH = -1; // meters // TODO: Value isn't set.
        public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI; // TODO: Check that this is accurate

        public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(-1); // TODO: Value isn't set.
        public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(-1); // TODO: Value isn't set.
        public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(-1); // TODO: Value isn't set.
        public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(-1); // TODO: Value isn't set.

        public static final double TRANSLATE_POWER_FAST = 1.0; // Scaling for teleop driving. 1.0 is maximum
        public static final double ROTATE_POWER_FAST = 0.75; // Scaling for teleop driving. 1.0 is maximum
        public static final double TRANSLATE_POWER_SLOW = 0.15; // Scaling for teleop driving. 1.0 is maximum
        public static final double ROTATE_POWER_SLOW = 0.15; // Scaling for teleop driving. 1.0 is maximum

        public static final double MAX_VOLTAGE = 12.0;
        public static final double TELEOP_CURRENT_LIMIT = -1; // TODO: Value isn't set.
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
    }

    public final class PIGEON {
        public static final int CAN_ID = 34;
    }
    public class ROBOT {
        public static final double JOYSTICK_DEADBAND = 0.01;
        private static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
    }
}
