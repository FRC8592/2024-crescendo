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
        public static final double ANGLE_DEGREES_TO_TICKS = 4096/360.0;
        public static final double METERS_TO_FEET = 3.28084;
        public static final double TICKS_TO_ANGLE_DEGREES = 360.0/4096.0;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }
    
    public final class CAN {
        
    }

    public final class NOTELOCK {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/NoteLock/";
        public static final String LIMELIGHT_NAME = "limelight-vision"; // TODO: Not set yet
        public static final double LOCK_ERROR = -1; // TODO: Not set yet
        public static final double CLOSE_ERROR = -1; // TODO: Not set yet
        public static final double CAMERA_HEIGHT = -1; // TODO: Not set yet
        public static final double DRIVE_TO_TARGET_ANGLE = -24;
        
        // Turn-to PID constants for the drive-to-note function
        public static final double DRIVE_TO_TURN_kP = 0.1;
        public static final double DRIVE_TO_TURN_kI = 0;
        public static final double DRIVE_TO_TURN_kD = 0;

        // Drive PID constants for the drive-to-note function
        public static final double DRIVE_TO_DRIVE_kP = 0.13;
        public static final double DRIVE_TO_DRIVE_kI = 0;
        public static final double DRIVE_TO_DRIVE_kD = 0;
    }

    public final class INTAKE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";
        public static final int TOP_MOTOR_CAN_ID = 35;
        public static final int BOTTOM_MOTOR_CAN_ID = 34;
        public static final double TOP_MOTOR_kP = 0.0001;
        public static final double TOP_MOTOR_kI = 0.000;
        public static final double TOP_MOTOR_kD = 0.000;
        public static final double BOTTOM_MOTOR_kP = 0.0001;
        public static final double BOTTOM_MOTOR_kI = 0.000;
        public static final double BOTTOM_MOTOR_kD = 0.000;
        public static final double MINIMUM_ROLLER_SPEED = 0;
        public static final double ROBOT_SPEED_MULTIPLIER = 100;
        public static final double SPEED_TOP = 1000;
        public static final double SPEED_BOTTOM = 1000;
    }

    public final class SHOOTER {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Shooter/";
        public static final double AMP_SPEED = 0;

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
        public static final int EXTENSION_MOTOR_CAN_ID = -1;
        public static final double POSITION_STOWED = -1;
        public static final double POSITION_AMP= -1;
        public static final int PIVOT_MOTOR_CAN_ID = -1;    
        public static final int PIVOT_FOLLOW_MOTOR_CAN_ID = -1;    
        public static final int PIVOT_ANGLE_AMP = -1;
        public static final int PIVOT_ANGLE_STOWED = -1;
        public static final int RETRACTED = 5; //Ticks TODO: Change to new unit once we know the gearing
        public static final int LIFTED = 30; //Degrees
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

        public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(233.525);
        public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(133.77);
        public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(78.75);
        public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(147.041);

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

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / // 6.0
                Math.hypot(SWERVE.DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0);
    }

    public final class PIGEON {
        public static final int CAN_ID = 17;
    }

    public class ROBOT {
        public static final double JOYSTICK_DEADBAND = 0.01;
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
    }
}
