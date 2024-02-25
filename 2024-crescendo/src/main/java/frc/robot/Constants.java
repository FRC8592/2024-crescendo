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

    public final class NOTELOCK {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/NoteLock/";
        public static final String LIMELIGHT_NAME = "limelight-vision"; // TODO: Not set yet
        public static final double LOCK_ERROR = -1; // TODO: Not set yet
        public static final double CLOSE_ERROR = -1; // TODO: Not set yet
        public static final double CAMERA_HEIGHT = -1; // TODO: Not set yet
        public static final double DRIVE_TO_TARGET_ANGLE = -20;
        
        // Turn-to PID constants for the drive-to-note function
        public static final double DRIVE_TO_TURN_kP = 0.075;
        public static final double DRIVE_TO_TURN_kI = 0;
        public static final double DRIVE_TO_TURN_kD = 0;

        // Drive PID constants for the drive-to-note function
        public static final double DRIVE_TO_DRIVE_kP = 0.13;
        public static final double DRIVE_TO_DRIVE_kI = 0;
        public static final double DRIVE_TO_DRIVE_kD = 0;

        //Turn PID constants (NOT drive-to)
        public static final double TURN_kP = 0.03;
        public static final double TURN_kI = 0;
        public static final double TURN_kD = 0;
    }

    public final class INTAKE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";
        public static final int TOP_MOTOR_CAN_ID = 35;
        public static final int BOTTOM_MOTOR_CAN_ID = 34;

        public static final double TOP_MOTOR_kP = 0.0007;
        public static final double TOP_MOTOR_kI = 0.00;
        public static final double TOP_MOTOR_kD = 0.01;
        public static final double TOP_MOTOR_kFF = 0.000175;

        // public static final double SPEED_TOP = 1000; TODO: Currently in power mode. We should change back to velocity mode before our first regional
        public static final double INTAKE_POWER = 0.75; //TODO: Delete this once we have velocity mode working
        public static final double OUTAKE_POWER = -0.75;

        // NOTE: only one motor on now
        // public static final double BOTTOM_MOTOR_kP = 0.0001;
        // public static final double BOTTOM_MOTOR_kI = 0.000;
        // public static final double BOTTOM_MOTOR_kD = 0.000;

    }

    public final class SHOOTER {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Shooter/";
        
        public static final double FEEDER_MOTOR_kP = 0; //TODO: Tune this; start with 0.00001;
        public static final double FEEDER_MOTOR_kI = 0.0;
        public static final double FEEDER_MOTOR_kD = 0.0;
        public static final double FEEDER_MOTOR_kF = 0.0001;

        public static final double TOP_SHOOTER_MOTOR_kP = 0.0000825;
        public static final double TOP_SHOOTER_MOTOR_kI = 0.0000001;
        public static final double TOP_SHOOTER_MOTOR_kD = 0.0000001;
        public static final double TOP_SHOOTER_MOTOR_kF = 0.000149; // feed-forward

        public static final double BOTTOM_SHOOTER_MOTOR_kP = TOP_SHOOTER_MOTOR_kP;
        public static final double BOTTOM_SHOOTER_MOTOR_kI = TOP_SHOOTER_MOTOR_kI;
        public static final double BOTTOM_SHOOTER_MOTOR_kD = TOP_SHOOTER_MOTOR_kD;
        public static final double BOTTOM_SHOOTER_MOTOR_kF = TOP_SHOOTER_MOTOR_kF;

        public static final double SHOOTER_MOTOR_IZONE = 100; // RPM for the range within which the I term will be effective

        public static final int FHYWHEEL_SPEED_ACCEPTABLE_RANGE = 100; // RPM

        public static final int ACCEPTABLE_RANGE = 35;
        //Lookup table
        public static final double[][] RANGE_TABLE = {
            {0,3000}
        };
        public static final int TOP_SHOOTER_MOTOR_CAN_ID = 29;
        public static final int BOTTOM_SHOOTER_MOTOR_CAN_ID = 28;
        public static final int FEEDER_MOTOR_CAN_ID = 31;
        public static final int NOTE_BEAM_BREAK_DIO_PORT = 0;

        public static final int AMP_SHOOTER_SPEED = -1000;
        public static final int AMP_FEEDER_SPEED = -2000; //TODO: Figure out what this should actually be

        public static final int INTAKE_SHOOTER_SPEED = 0;
        public static final int INTAKE_FEEDER_SPEED = 4000;

        //No shooter speed because we use the range table
        public static final int SHOOTING_FEEDER_SPEED = 1500;

        public static final int OUTAKE_SHOOTER_VELOCITY = -1; //TODO: Figure out what this should be.
        public static final double OUTAKE_FEEDER_VELOCITY = -2000;
    }


    public final class ELEVATOR {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Elevator/";

        public static final int EXTENSION_MOTOR_CAN_ID = 37;
        public static final int PIVOT_MOTOR_CAN_ID = 36;
        public static final int PIVOT_FOLLOW_MOTOR_CAN_ID = 38; 

        public static final double PIVOT_kP = 0.000001;
        public static final double PIVOT_kI = 0;
        public static final double PIVOT_kD = 0;
        public static final double PIVOT_kFF = 0.00025;

        public static final double EXTENSION_kP = 0.000001;
        public static final double EXTENSION_kI = 0.0;
        public static final double EXTENSION_kD = 0.0;
        public static final double EXTENSION_kFF = 0.00025;

        public static final double EXTENSION_METERS_MAX = 0.26; // TODO: Figure out what this should be
        public static final int PIVOT_ANGLE_MAX = 75; // TODO: Figure out what this should be

        public static final double EXTENSION_METERS_MIN = 0;
        public static final int PIVOT_ANGLE_MIN = 0;

        public static final double EXTENSION_METERS_STOWED = 0;
        public static final int PIVOT_ANGLE_STOWED = 0;

        public static final double EXTENSION_METERS_AMP = 0.27;
        public static final int PIVOT_ANGLE_AMP = 48; // TODO: Adjust this angle

        public static final double EXTENSION_METERS_CLIMB = 0.25;
        public static final int PIVOT_ANGLE_CLIMB = 75; // TODO: Adjust this angle

        public static final double EXTENSION_FULLY_RETRACTED = 0.001; //Meters
        public static final double EXTENSION_FORCE_RETRACT_THRESHOLD = 30; //Degrees

        public static final double PIVOT_GEAR_RATIO = (12.0 / 5.0) * 75.0; // 12/5 is the radio of the large pulley to small pulley. 75 is from the gearbox.

        public static final double DIAMETER_OF_ELEVATOR_SPROCKET = 1.885; //inches
        public static final double ELEVATOR_GEAR_RATIO =
                (1 / 48.0) *                                //Multiply by the gearbox ratio,
                DIAMETER_OF_ELEVATOR_SPROCKET * Math.PI *   //then multiply by πd (same as 2πr) to get circumference in inches,
                CONVERSIONS.IN_TO_METERS;                   //then convert to meters.
        public static final double MAX_PIVOT_ROTATIONS = (PIVOT_ANGLE_MAX*PIVOT_GEAR_RATIO)/360;
        public static final double MAX_EXTENSION_ROTATIONS = EXTENSION_METERS_MAX / ELEVATOR_GEAR_RATIO;

        public static final double MANUAL_EXTENSION_SPEED = 0.005; //TODO: Adjust this
        public static final double MANUAL_PIVOT_SPEED = 0.1;

        public static final double RETRACT_THRESHOLD_TOLERANCE = 2;

        public static final double ANGLE_TOLERANCE = 0.5;
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

        public static final double BLACK_FRONT_LEFT_STEER_OFFSET = -Math.toRadians(320.098); // 354.639
        public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(161.455); // 0
        public static final double TEAL_BACK_LEFT_STEER_OFFSET = -Math.toRadians(243.809);    // 69.434
        public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -Math.toRadians(178.857); // 6.768

        public static final double TRANSLATE_POWER_FAST = 1.0; // Scaling for teleop driving. 1.0 is maximum
        public static final double ROTATE_POWER_FAST = 0.75; // Scaling for teleop driving. 1.0 is maximum
        public static final double TRANSLATE_POWER_SLOW = 0.25; // Scaling for teleop driving. 1.0 is maximum
        public static final double ROTATE_POWER_SLOW = 0.2; // Scaling for teleop driving. 1.0 is maximum

        public static final double MAX_VOLTAGE = 12.0;
        public static final double TELEOP_CURRENT_LIMIT = 15.0; 
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / // 6.0
                Math.hypot(SWERVE.DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0);
    }

    public final class PIGEON {
        public static final int CAN_ID = 17;
    }

    public final class ROBOT {
        public static final double JOYSTICK_DEADBAND = 0.01;
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
    }

    public class APRILTAG_VISION {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }
}
