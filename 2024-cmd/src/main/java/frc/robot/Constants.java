package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
    public final class SHARED {
        public static final String LOG_FOLDER = "CustomLogs";
    }

    public final class CONVERSIONS {
        // TODO: Figure this out
        public static final double METERS_SECOND_TO_TICKS_TALONFX = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

        public static final double RPM_TO_TICKS_100_MS_TALONFX = 2048.0 / 600.0;

        public static final double ANGLE_DEGREES_TO_TICKS_SPARKFLEX = 4096 / 360.0;
        public static final double TICKS_TO_ANGLE_DEGREES_SPARKFLEX = 360.0 / 4096.0;

        public static final double DEG_TO_RAD = 0.0174533;
        public static final double RAD_TO_DEG = 57.2958;
        public static final double IN_TO_METERS = 0.0254;
        public static final double METERS_TO_FEET = 3.28084;
    }

    public final class MEASUREMENTS {
        public static final double FIELD_LENGTH_METERS = 16.5410515;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public final class CAN {
        public static final int INTAKE_MOTOR_CAN_ID = 35;

        public static final int TOP_SHOOTER_MOTOR_CAN_ID = 29;
        public static final int BOTTOM_SHOOTER_MOTOR_CAN_ID = 28;
        public static final int FEEDER_MOTOR_CAN_ID = 31;

        public static final int ELEVATOR_MOTOR_CAN_ID = 37;
        public static final int PIVOT_MOTOR_CAN_ID = 36;
        public static final int PIVOT_FOLLOW_MOTOR_CAN_ID = 38;

        public static final int SWERVE_BLACK_FRONT_LEFT_DRIVE_CAN_ID = 14;
        public static final int SWERVE_BLACK_FRONT_LEFT_STEER_CAN_ID = 13;
        public static final int SWERVE_BLACK_FRONT_LEFT_ENCODER_CAN_ID = 15;

        public static final int SWERVE_ORANGE_FRONT_RIGHT_DRIVE_CAN_ID = 20;
        public static final int SWERVE_ORANGE_FRONT_RIGHT_STEER_CAN_ID = 19;
        public static final int SWERVE_ORANGE_FRONT_RIGHT_ENCODER_CAN_ID = 21;

        public static final int SWERVE_TEAL_BACK_LEFT_DRIVE_CAN_ID = 11;
        public static final int SWERVE_TEAL_BACK_LEFT_STEER_CAN_ID = 10;
        public static final int SWERVE_TEAL_BACK_LEFT_ENCODER_CAN_ID = 12;

        public static final int SWERVE_WHITE_BACK_RIGHT_DRIVE_CAN_ID = 17;
        public static final int SWERVE_WHITE_BACK_RIGHT_STEER_CAN_ID = 16;
        public static final int SWERVE_WHITE_BACK_RIGHT_ENCODER_CAN_ID = 18;

        public static final int PIGEON_CAN_ID = 17;

        public static final int PDH_CAN_ID = 1;
    }

    public final class POWER {
        public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;

        public static final int LEFT_SHOOTER_MOTOR_CURRENT_LIMIT = 60;
        public static final int RIGHT_SHOOTER_MOTOR_CURRENT_LIMIT = 60;
        public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;

        public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 60;
        public static final int PIVOT_MOTOR_CURRENT_LIMIT = 40;
        public static final int PIVOT_FOLLOW_MOTOR_CURRENT_LIMIT = 40;

        public static final int SWERVE_MAX_VOLTAGE = 12;
        public static final int SWERVE_DRIVE_CURRENT_LIMIT = 80;
        public static final int SWERVE_STEER_CURRENT_LIMIT = 40;
    }

    public final class NOTELOCK {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/NoteLock/";

        public static final String LIMELIGHT_NAME = "limelight-vision";

        // These don't do anything
        public static final double LOCK_ERROR = -1;
        public static final double CLOSE_ERROR = -1;
        public static final double CAMERA_HEIGHT = -1;

        // Turn-to PID constants for the drive-to-note function
        public static final double DRIVE_TO_TURN_kP = 0.05;
        public static final double DRIVE_TO_TURN_kI = 0;
        public static final double DRIVE_TO_TURN_kD = 0;

        // Drive PID constants for the drive-to-note function
        public static final double DRIVE_TO_DRIVE_kP = 0.2;
        public static final double DRIVE_TO_DRIVE_kI = 0.3;
        public static final double DRIVE_TO_DRIVE_kD = 0;

        public static final double AUTO_DRIVE_TO_TARGET_ANGLE = -24;
        public static final double TELEOP_DRIVE_TO_TARGET_ANGLE = -24;
    }

    public final class INTAKE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";

        public static final double MOTOR_kP = 0.0003;
        public static final double MOTOR_kI = 0.00;
        public static final double MOTOR_kD = 0.015;
        public static final double MOTOR_kFF = 0.00016;

        public static final double INTAKE_VELOCITY = 4500;
        public static final double OUTAKE_VELOCITY = -2500;
    }

    public final class SHOOTER {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Shooter/";

        public static final int BOTTOM_BEAM_BREAK_DIO_PORT = 0;
        public static final int TOP_BEAM_BREAK_DIO_PORT = 1;
        public static final int MIDDLE_BEAM_BREAK_DIO_PORT = 2;

        // Run when the note is not in contact with the feeder
        public static final double FEEDER_MOTOR_kP = 0.00001;
        public static final double FEEDER_MOTOR_kI = 0.0;
        public static final double FEEDER_MOTOR_kD = 0.0;
        public static final double FEEDER_MOTOR_kF = 0.00018;

        // Alternate constants to use while in contact with the note. Used for
        // the alignment routine that runs after the note has entered the robot.
        public static final double FEEDER_MOTOR_LOADED_kP = 0.001;
        public static final double FEEDER_MOTOR_LOADED_kI = 0.0000005;
        public static final double FEEDER_MOTOR_LOADED_kD = 0.01;
        public static final double FEEDER_MOTOR_LOADED_kF = 0.00018;
        public static final double FEEDER_MOTOR_LOADED_IZONE = 160;

        public static final double FEEDER_MOTOR_OUTAKE_kP = 0.001;
        public static final double FEEDER_MOTOR_OUTAKE_kI = 0.00001;
        public static final double FEEDER_MOTOR_OUTAKE_kD = 0.01;
        public static final double FEEDER_MOTOR_OUTAKE_kF = 0.00018;
        public static final double FEEDER_MOTOR_OUTAKE_IZONE = 160;

        public static final double LEFT_SHOOTER_MOTOR_kP = 0.0003;
        public static final double LEFT_SHOOTER_MOTOR_kI = 0.0000012;
        public static final double LEFT_SHOOTER_MOTOR_kD = 0.01;
        public static final double LEFT_SHOOTER_MOTOR_kF = 0.000155;

        public static final double RIGHT_SHOOTER_MOTOR_kP = LEFT_SHOOTER_MOTOR_kP;
        public static final double RIGHT_SHOOTER_MOTOR_kI = LEFT_SHOOTER_MOTOR_kI;
        public static final double RIGHT_SHOOTER_MOTOR_kD = LEFT_SHOOTER_MOTOR_kD;
        public static final double RIGHT_SHOOTER_MOTOR_kF = LEFT_SHOOTER_MOTOR_kF;

        // Tolerance of the target RPM within which the I term accumulate and affect the output
        public static final double SHOOTER_MOTOR_IZONE = 200;

        // RPM tolerance before shooting
        public static final int FLYWHEEL_SPEED_ACCEPTABLE_RANGE = 50;

        public static final int AMP_FLYWHEEL_SPEED = -1000;
        public static final int AMP_FEEDER_SPEED = -3000;

        public static final int INTAKE_FLYWHEEL_SPEED = 0;
        public static final int INTAKE_FEEDER_SPEED = 2000; // When the note is not in contact with the feeder
        public static final double INTAKE_FEEDER_POWER = 1d; // Used between when the note contacts the feeder and when it reaches the flywheels

        // public static final int SHOOTING_FEEDER_SPEED = 2500;
        public static final double SHOOTING_FEEDER_POWER = 1;

        public static final int OUTAKE_FLYWHEEL_SPEED = -500;
        public static final int OUTAKE_FEEDER_SPEED = -2000;

        public static final int PASSTHROUGH_SHOOTER_SPEED = 5000;
        public static final double PASSTHROUGH_FEEDER_POWER = 1;

        public static final double SHOOT_SCORE_TIME = 0.5;
        public static final double AMP_SCORE_TIME = 2;

        public static final int STAGE_FEEDER_SPEED = 250;
        public static final int ALIGN_FEEDER_SPEED = -150;
        public static final int ALIGN_FLYWHEEL_SPEED = -150;
    }


    public final class ELEVATOR {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Elevator/";

        public static final double PIVOT_kP = 0.000000375;
        public static final double PIVOT_kI = 0;
        public static final double PIVOT_kD = 0.000001;
        public static final double PIVOT_kFF = 0.00025;
        public static final double PIVOT_IZONE = 1.0; // angle in deg

        public static final double EXTENSION_kP = 0.000001;
        public static final double EXTENSION_kI = 0.0;
        public static final double EXTENSION_kD = 0.0;
        public static final double EXTENSION_kFF = 0.00025;

        public static final int EXTENSION_MAX_VELOCITY = 5000;
        public static final int EXTENSION_MAX_ACCELERATION = 10000;
        public static final int PIVOT_MAX_VELOCITY = 6500;
        public static final int PIVOT_MAX_ACCELERATION = 7000;

        // When the extension is retracted below this many meters, we can safetly lower the pivot to the home position
        public static final double EXTENSION_FULLY_RETRACTED = 0.01;

        // The angle the pivot will be kept at if the extension is above EXTENSION_FULLY_RETRACTED meters
        public static final double EXTENSION_FORCE_RETRACT_THRESHOLD = 20;

        // Tolerance for being "at EXTENSION_FORCE_RETRACT_THRESHOLD degrees"
        public static final double RETRACT_THRESHOLD_TOLERANCE = 2;

        // Degree tolerance for angle setpoints before claiming to be at the setpoint
        public static final double ANGLE_TOLERANCE = 0.1;

        // Meter tolerance for extension setpoints before claiming to be at the setpoint
        public static final double LENGTH_TOLERANCE = 0.005;

        public static final double EXTENSION_METERS_MAX = 0.279;
        public static final int PIVOT_ANGLE_MAX = 75;

        public static final double EXTENSION_METERS_MIN = 0;
        public static final int PIVOT_ANGLE_MIN = 0;

        public static final double EXTENSION_METERS_STOWED = 0;
        public static final int PIVOT_ANGLE_STOWED = 0;

        public static final double EXTENSION_METERS_AMP = 0.27;
        public static final int PIVOT_ANGLE_AMP = 45;

        public static final double EXTENSION_METERS_CLIMB = 0.279;
        public static final int PIVOT_ANGLE_CLIMB = 75;

        public static final double MANUAL_EXTENSION_SPEED = 0.005;

        public static final double PIVOT_GEAR_RATIO = (
            (12.0 / 5.0) // 12/5 is the ratio of the large pulley to small pulley
            * 75.0       // 75 is from the gearbox
        );

        public static final double DIAMETER_OF_ELEVATOR_SPROCKET = 1.885; //inches
        public static final double ELEVATOR_GEAR_RATIO = (
            (1 / 48.0)                                // Gearbox ratio
            * DIAMETER_OF_ELEVATOR_SPROCKET * Math.PI // Multiply by πd (same as 2πr) to get circumference in inches
            * CONVERSIONS.IN_TO_METERS                // Convert to meters
        );
    }


    public final class LEDS {
        public static final int LED_LENGTH = 30;
        public static final int LED_STRIP_PWM_PORT = 0;

        public static final Color RED = new Color(255, 0, 0);
        public static final Color GREEN = new Color(0, 255, 0);
        public static final Color BLUE = new Color(0, 0, 255);
        public static final Color YELLOW = new Color(255, 255, 0);
        public static final Color CYAN = new Color(0, 192, 255);
        public static final Color MAGENTA = new Color(255, 0, 255);
        public static final Color ORANGE = new Color(255, 64, 0);
        public static final Color OFF = new Color(0, 0, 0);
        public static final Color WHITE = new Color(255, 255, 255);

        public static final double NOT_AIMED_OFFSET = 2;
        public static final double FULLY_AIMED_OFFSET = APRILTAG_VISION.X_ROT_LOCK_TOLERANCE;
    }

    public final class SWERVE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Swerve/";

        public static final double STEER_P = 100;
        public static final double STEER_I = 0;
        public static final double STEER_D = 0.2;
        public static final double STEER_S = 0;
        public static final double STEER_V = 1.5;
        public static final double STEER_A = 0;

        public static final double DRIVE_P = 3;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
        public static final double DRIVE_S = 0;
        public static final double DRIVE_V = 0;
        public static final double DRIVE_A = 0;

        public static final double SNAP_TO_kP = 3.7;
        public static final double SNAP_TO_kI = 0.0;
        public static final double SNAP_TO_kD = 0.1;

        public static final int STEER_STATOR_CURRENT_LIMIT = 60;

        public static final int CALCULATED_SLIP_CURRENT = 150;

        public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = 4.73;
        public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720);
        public static final double COUPLING_GEAR_RATIO = 3.5714285714285716;
        public static final double DRIVE_GEAR_RATIO = 6.746031746031747;
        public static final double STEER_GEAR_RATIO = 21.428571428571427;
        public static final double WHEEL_RADIUS_INCHES = 2;

        public static final boolean INVERT_LEFT_SIDE = false;
        public static final boolean INVERT_RIGHT_SIDE = true;

        public static final double SIMULATED_STEER_INERTIA = 0.00001;
        public static final double SIMULATED_DRIVE_INERTIA = 0.06;
        public static final double SIMULATION_LOOP_PERIOD = 0.005;
        public static final double STEER_FRICTION_VOLTAGE = 0.25;
        public static final double DRIVE_FRICTION_VOLTAGE = 0.25;

        // Scaling for teleop driving. 1.0 is maximum.
        public static final double TRANSLATE_POWER_FAST = 1.0;
        public static final double ROTATE_POWER_FAST = 0.75;
        public static final double TRANSLATE_POWER_SLOW = 0.15;
        public static final double ROTATE_POWER_SLOW = 0.15;

        public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
        public static final int ROTATION_SMOOTHING_AMOUNT = 1;

        public static final double JOYSTICK_EXPONENT = 2;

        public static final Rotation2d BLUE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
        public static final Rotation2d RED_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);

        // public static final double BLACK_FRONT_LEFT_STEER_OFFSET = -0.388427734375;
        // public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -0.462646484375;
        // public static final double TEAL_BACK_LEFT_STEER_OFFSET = -0.18017578125;
        // public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -0.4853515625;

        public static final double BLACK_FRONT_LEFT_STEER_OFFSET = -0.389648;
        public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -0.441162;
        public static final double TEAL_BACK_LEFT_STEER_OFFSET = -0.198730;
        public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -0.498291;

        public static final boolean BLACK_FRONT_LEFT_STEER_INVERT = true;
        public static final boolean ORANGE_FRONT_RIGHT_STEER_INVERT = true;
        public static final boolean TEAL_BACK_LEFT_STEER_INVERT = true;
        public static final boolean WHITE_BACK_RIGHT_STEER_INVERT = true;


        public static final double BLACK_FRONT_LEFT_X_POSITION = 10.25;
        public static final double BLACK_FRONT_LEFT_Y_POSITION = 9.25;

        public static final double ORANGE_FRONT_RIGHT_X_POSITION = 10.25;
        public static final double ORANGE_FRONT_RIGHT_Y_POSITION = -9.25;

        public static final double TEAL_BACK_LEFT_X_POSITION = -10.25;
        public static final double TEAL_BACK_LEFT_Y_POSITION = 9.25;

        public static final double WHITE_BACK_RIGHT_X_POSITION = -10.25;
        public static final double WHITE_BACK_RIGHT_Y_POSITION = -9.25;

        public static final double PATH_FOLLOW_TRANSLATE_kP = 6d;
        public static final double PATH_FOLLOW_TRANSLATE_kI = 0d;
        public static final double PATH_FOLLOW_TRANSLATE_kD = 0d;

        public static final double PATH_FOLLOW_ROTATE_kP = 3.7;
        public static final double PATH_FOLLOW_ROTATE_kI = 0d;
        public static final double PATH_FOLLOW_ROTATE_kD = 0.1;

        public static final double PATH_FOLLOW_ROTATE_MAX_VELOCITY = 4 * Math.PI;
        public static final double PATH_FOLLOW_ROTATE_MAX_ACCELLERATION = 2 * Math.PI;

        public static final double PATH_FOLLOW_POSITION_TOLERANCE = 0.1;
        public static final double PATH_FOLLOW_VELOCITY_TOLERANCE = 0.1;
    }

    public final class ROBOT {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
    }

    public class APRILTAG_VISION {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/MGVision/";
        public static final double kP = 0.9;
        public static final double kI = 6;
        public static final double kD = 0.085;
        public static final double iZONE = 0.25;

        // red, blue
        public static final ArrayList<Integer> SPEAKER_AIM_TAGS = new ArrayList<>(Arrays.asList(4, 7));
        public static final ArrayList<Integer> AMP_AIM_TAGS = new ArrayList<>(Arrays.asList(5, 6));
        public static final ArrayList<Integer> SOURCE_AIM_TAGS = new ArrayList<>(Arrays.asList(1,2,9,10));
        public static final ArrayList<Integer> STAGE_AIM_TAGS = new ArrayList<>(Arrays.asList(11, 12, 13, 14, 15, 16));

        public static final double X_ROT_LOCK_TOLERANCE = 0.075;
    }

    public class RANGE_TABLE{
        public static final double OFFSET_ANGLE = 0.5; // prev. -0.5
    }

    public class SUPPLIERS{
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Suppliers/";
    }
}
