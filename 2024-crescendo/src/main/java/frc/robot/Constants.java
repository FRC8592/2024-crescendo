package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import frc.robot.NeoPixelLED.NewtonColor;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public final class SHARED {
        public static final String LOG_FOLDER = "CustomLogs";
    }

    public final class CONVERSIONS {
        public static final double METERS_SECOND_TO_TICKS_TALONFX = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));
        public static final double RPM_TO_TICKS_100_MS_TALONFX = 2048.0 / 600.0;

        public static final double ANGLE_DEGREES_TO_TICKS_SPARKFLEX = 4096 / 360.0;
        public static final double TICKS_TO_ANGLE_DEGREES_SPARKFLEX = 360.0 / 4096.0;

        public static final double DEG_TO_RAD = 0.0174533;
        public static final double RAD_TO_DEG = 57.2958;
        public static final double IN_TO_METERS = 0.0254;
        public static final double METERS_TO_FEET = 3.28084;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public final class CAN{
        public static final int INTAKE_MOTOR_CAN_ID = 35;

        public static final int TOP_SHOOTER_MOTOR_CAN_ID = 29;
        public static final int BOTTOM_SHOOTER_MOTOR_CAN_ID = 28;
        public static final int FEEDER_MOTOR_CAN_ID = 31;

        public static final int ELEVATOR_MOTOR_CAN_ID = 37;
        public static final int PIVOT_MOTOR_CAN_ID = 36;
        public static final int PIVOT_FOLLOW_MOTOR_CAN_ID = 38;

        public static final int PDH_CAN_ID = 1;

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
    }

    public final class POWER {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Power/";

        public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;

        public static final int LEFT_SHOOTER_MOTOR_CURRENT_LIMIT = 60;
        public static final int RIGHT_SHOOTER_MOTOR_CURRENT_LIMIT = 60;
        public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;

        public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 60;
        public static final int PIVOT_MOTOR_CURRENT_LIMIT = 40;
        public static final int PIVOT_FOLLOW_MOTOR_CURRENT_LIMIT = 40;

        public static final int SWERVE_MAX_VOLTAGE = 12;
        public static final int SWERVE_TELEOP_THROTTLE_CURRENT_LIMIT = 80;
        public static final int SWERVE_AUTO_THROTTLE_CURRENT_LIMIT = 60;
        public static final int SWERVE_AZIMUTH_CURRENT_LIMIT = 40;

        public static final int VOLTAGE_SMOOTHING_LENGTH = 50; // TODO: Arbitrary (needs testing)
        public static final double DISABLED_LOW_BATTERY_VOLTAGE = 11.5; // TODO: Arbitrary (needs testing)
        public static final double TELEOP_LOW_BATTERY_VOLTAGE = 10.5; // TODO: Arbitrary (needs testing)
    }

    public final class NOTELOCK {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/NoteLock/";

        public static final String LIMELIGHT_NAME = "limelight-vision";

        public static final double LOCK_ERROR = -1; // TODO: Not set yet
        public static final double CLOSE_ERROR = -1; // TODO: Not set yet
        public static final double CAMERA_HEIGHT = -1; // TODO: Not set yet

        // Turn-to PID constants for the drive-to-note function
        public static final double DRIVE_TO_TURN_kP = 0.05;
        public static final double DRIVE_TO_TURN_kI = 0;
        public static final double DRIVE_TO_TURN_kD = 0;

        // Drive PID constants for the drive-to-note function
        public static final double DRIVE_TO_DRIVE_kP = 0.13;
        public static final double DRIVE_TO_DRIVE_kI = 0;
        public static final double DRIVE_TO_DRIVE_kD = 0.01;

        public static final double AUTO_DRIVE_TO_TARGET_ANGLE = -24;
        public static final double TELEOP_DRIVE_TO_TARGET_ANGLE = -24;
        public static final double DY_LIMIT = -20;

        public static final double TURN_kP = 0.0075;
        public static final double TURN_kI = 0;
        public static final double TURN_kD = 0;
    }

    //TODO: None of this class should be needed anymore
    public final class APRILTAG_LIMELIGHT {
        public static final String LIMELIGHT_NAME = "limelight-target"; // TODO idk what this is
        
        // for auto lock to speaker
        public static final double SPEAKER_TURN_kP = 0.05;
        public static final double SPEAKER_TURN_kI = 0.032;
        public static final double SPEAKER_TURN_kD = 0;
        public static final double SPEAKER_TURN_IZONE = 5;

        public static final double SPEAKER_DRIVE_kP = 0.2;
        public static final double SPEAKER_DRIVE_kI = 0.3;
        public static final double SPEAKER_DRIVE_kD = 0;
        public static final double SPEAKER_TY_TARGET = 13.5;

        public static final double LOCK_ERROR = 0.1;
        public static final double CLOSE_ERROR = 0.1;
        public static final double CAMERA_HEIGHT = 0;
        public static final double CAMERA_ANGLE = 0;
        public static final double TARGET_HEIGHT = 0;
    }

    public final class INTAKE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";

        public static final double MOTOR_kP = 0.0003;
        public static final double MOTOR_kI = 0.00; //TODO: We need a lot of this
        public static final double MOTOR_kD = 0.015;
        public static final double MOTOR_kFF = 0.00016;

        // public static final double SPEED_TOP = 1000; TODO: Currently in power mode. We should change back to velocity mode before our first regional
        public static final double INTAKE_POWER = 0.75; //TODO: Delete this once we have velocity mode working
        public static final double OUTAKE_POWER = -0.75;

        public static final double INTAKE_VELOCITY = 4500;
        public static final double OUTAKE_VELOCITY = -2500;

        // NOTE: only one motor on now
        // public static final double BOTTOM_MOTOR_kP = 0.0001;
        // public static final double BOTTOM_MOTOR_kI = 0.000;
        // public static final double BOTTOM_MOTOR_kD = 0.000;

    }

    public final class SHOOTER {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Shooter/";

        public static final int BOTTOM_BEAM_BREAK_DIO_PORT = 0;
        public static final int TOP_BEAM_BREAK_DIO_PORT = 1;
        public static final int MIDDLE_BEAM_BREAK_DIO_PORT = 2;

        public static final double FEEDER_MOTOR_kP = 0.00001;
        public static final double FEEDER_MOTOR_kI = 0.0;
        public static final double FEEDER_MOTOR_kD = 0.0;
        public static final double FEEDER_MOTOR_kF = 0.00018;

        public static final double FEEDER_MOTOR_LOADED_kP = 0.001;
        public static final double FEEDER_MOTOR_LOADED_kI = 0.0000005;//1;
        public static final double FEEDER_MOTOR_LOADED_kD = 0.01;
        public static final double FEEDER_MOTOR_LOADED_kF = 0.00018;
        public static final double FEEDER_MOTOR_LOADED_IZONE = 160;

        public static final double FEEDER_MOTOR_OUTAKE_kP = 0.001;
        public static final double FEEDER_MOTOR_OUTAKE_kI = 0.00001;//1;
        public static final double FEEDER_MOTOR_OUTAKE_kD = 0.01;
        public static final double FEEDER_MOTOR_OUTAKE_kF = 0.00018;
        public static final double FEEDER_MOTOR_OUTAKE_IZONE = 160;

        public static final double LEFT_SHOOTER_MOTOR_kP = 0.0003;
        public static final double LEFT_SHOOTER_MOTOR_kI = 0.0000012;
        public static final double LEFT_SHOOTER_MOTOR_kD = 0.01;
        public static final double LEFT_SHOOTER_MOTOR_kF = 0.000155; // feed-forward

        public static final double RIGHT_SHOOTER_MOTOR_kP = LEFT_SHOOTER_MOTOR_kP;
        public static final double RIGHT_SHOOTER_MOTOR_kI = LEFT_SHOOTER_MOTOR_kI;
        public static final double RIGHT_SHOOTER_MOTOR_kD = LEFT_SHOOTER_MOTOR_kD;
        public static final double RIGHT_SHOOTER_MOTOR_kF = LEFT_SHOOTER_MOTOR_kF;

        public static final double SHOOTER_MOTOR_IZONE = 200; // RPM for the range within which the I term will be effective


        public static final int FHYWHEEL_SPEED_ACCEPTABLE_RANGE = 50; // RPM. TODO: Decrease this when the PID is tuned better


        public static final int AMP_FLYWHEEL_SPEED = -1000;
        public static final int AMP_FEEDER_SPEED = -3000; //TODO: Figure out what this should actually be

        public static final int INTAKE_FLYWHEEL_SPEED = 0;
        public static final int INTAKE_FEEDER_SPEED = 2000;

        //No shooter speed because we use the range table
        // public static final int SHOOTING_FEEDER_SPEED = 2500;
        public static final double SHOOTING_FEEDER_POWER = 1d;

        public static final int OUTAKE_FLYWHEEL_SPEED = -500; //TODO: Figure out what this should be.
        public static final double OUTAKE_FEEDER_SPEED = -2000;


        public static final double SHOOT_SCORE_TIME = 0.5; //TODO: Tune this
        public static final double AMP_SCORE_TIME = 0.25; //TODO: Tune this

        public static final double FEEDER_AMP_TOLERANCE = 100;

        public static final double STAGE_FEEDER_SPEED = 250;
        public static final double ALIGN_FEEDER_SPEED = -150;
        public static final int ALIGN_FLYWHEEL_SPEED = -150;

        public static final double KEEP_RAMMING_TIME = 0.1;
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


        //Protection control config
        public static final double EXTENSION_FULLY_RETRACTED = 0.01; //When the extension is retracted below this value, we can lower the pivot to zero.
        public static final double EXTENSION_FORCE_RETRACT_THRESHOLD = 20; //The threshold for the protection code from the hooks on the elevator. See Elevator.java.
        public static final double RETRACT_THRESHOLD_TOLERANCE = 2; //If we're within this value of the threshold (or higher), the extension works.
        public static final double ANGLE_TOLERANCE = 0.1; //Half a degree; used for the function that detects whether the pivot is at the target angle.
        public static final double LENGTH_TOLERANCE = 0.005; //Half a centimeter; used for the function that detects whether the extension is at the target length.


        public static final double EXTENSION_METERS_MAX = 0.279; // TODO: Figure out what this should be
        public static final int PIVOT_ANGLE_MAX = 75; // TODO: Figure out what this should be

        public static final double EXTENSION_METERS_MIN = 0;
        public static final int PIVOT_ANGLE_MIN = 0;

        public static final double EXTENSION_METERS_STOWED = 0;
        public static final int PIVOT_ANGLE_STOWED = 0;

        public static final double EXTENSION_METERS_AMP = 0.27;
        public static final int PIVOT_ANGLE_AMP = 45; // TODO: Adjust this angle

        public static final double EXTENSION_METERS_CLIMB = 0.279;
        public static final int PIVOT_ANGLE_CLIMB = 75; // TODO: Adjust this angle


        public static final double MANUAL_EXTENSION_SPEED = 0.005; //TODO: Adjust this


        public static final double PIVOT_GEAR_RATIO = (12.0 / 5.0) * 75.0; // 12/5 is the ratio of the large pulley to small pulley. 75 is from the gearbox.

        public static final double DIAMETER_OF_ELEVATOR_SPROCKET = 1.885; //inches
        public static final double ELEVATOR_GEAR_RATIO = 
                (1 / 48.0) *                                //Multiply by the gearbox ratio,
                DIAMETER_OF_ELEVATOR_SPROCKET * Math.PI *   //then multiply by πd (same as 2πr) to get circumference in inches,
                CONVERSIONS.IN_TO_METERS;                   //then convert to meters.

        public static final double MAX_PIVOT_ROTATIONS = (PIVOT_ANGLE_MAX * PIVOT_GEAR_RATIO) / 360;
        public static final double MAX_EXTENSION_ROTATIONS = EXTENSION_METERS_MAX / ELEVATOR_GEAR_RATIO;
    }


    public final class LEDS { //TODO: Merge the LED code once we get the LEDs working physically
        public static final int LED_LENGTH = 30;
        public static final double INTAKING_TIMEOUT = 0.5; //Intake for this long before the LEDs start reporting a jam
        public static final NewtonColor RED = new NewtonColor(255, 0, 0);
        public static final NewtonColor GREEN = new NewtonColor(0, 255, 0);
        public static final NewtonColor BLUE = new NewtonColor(0, 0, 255);
        public static final NewtonColor YELLOW = new NewtonColor(255, 255, 0);
        public static final NewtonColor CYAN = new NewtonColor(0, 192, 255);
        public static final NewtonColor MAGENTA = new NewtonColor(255, 0, 255);
        public static final NewtonColor ORANGE = new NewtonColor(255, 64, 0);
        public static final NewtonColor OFF = new NewtonColor(0, 0, 0);
        public static final NewtonColor WHITE = new NewtonColor(255, 255, 255);

        public static final double NOT_AIMED_OFFSET = 2;
        public static final double FULLY_AIMED_OFFSET = APRILTAG_VISION.X_ROT_LOCK_ERROR;
    }

    public final class SWERVE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Swerve/";

        public static final double THROTTLE_kP = 0.02;
        public static final double THROTTLE_kI = 0.0;
        public static final double THROTTLE_kD = 0.01;

        public static final double STEER_kP = 0.2;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.1;

        public static final double SNAP_TO_kP = 3.2;
        public static final double SNAP_TO_kI = 0.0;
        public static final double SNAP_TO_kD = 0.1;


        public static final double DRIVE_TRAIN_WIDTH = 0.527; // meters
        public static final double DRIVE_TRAIN_LENGTH = 0.478; // meters 
        public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;


        public static final double BLACK_FRONT_LEFT_STEER_OFFSET = -Math.toRadians(320.977); // 145.02+180
        public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(159.17); // 339.785-180
        public static final double TEAL_BACK_LEFT_STEER_OFFSET = -Math.toRadians(249.961);    // 62.93+180 // 246.709
        public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -Math.toRadians(174.639); // 3.867+180


        public static final double TRANSLATE_POWER_FAST = 1.0; // Scaling for teleop driving. 1.0 is maximum
        public static final double ROTATE_POWER_FAST = 0.75; // Scaling for teleop driving. 1.0 is maximum
        public static final double TRANSLATE_POWER_SLOW = 0.25; // Scaling for teleop driving. 1.0 is maximum
        public static final double ROTATE_POWER_SLOW = 0.2; // Scaling for teleop driving. 1.0 is maximum


        // public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.95; // 4.5 + 0.1(4.5) to go from 5700 --> 6300
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.5; // crazy test
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(SWERVE.DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0);

        public static final int TRANSLATION_SMOOTHING_AMOUNT = 3; // TODO: Currently unset, tune this
        public static final int ROTATION_SMOOTHING_AMOUNT = 1; // TODO: Currently unset, tune this

        public static final double JOYSTICK_EXPONENT = 2;
    }

    public final class ROBOT {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
        public static final double JOYSTICK_DEADBAND = 0.01; //TODO: Implement this
    }

    public class APRILTAG_VISION {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/MGVision/";
        public static final double kP = 0.9;
        public static final double kI = 6; // was 4
        public static final double kD = 0.085; // was 0.005
        public static final double iZone = 0.25;

        // red, blue
        public static final ArrayList<Integer> SPEAKER_AIM_TAGS = new ArrayList<>(Arrays.asList(4, 7));
        public static final ArrayList<Integer> AMP_AIM_TAGS = new ArrayList<>(Arrays.asList(5, 6));
        public static final ArrayList<Integer> SOURCE_AIM_TAGS = new ArrayList<>(Arrays.asList(1,2,9,10));
        public static final ArrayList<Integer> STAGE_AIM_TAGS = new ArrayList<>(Arrays.asList(11, 12, 13, 14, 15, 16));

        public static final double FUSE_DISTANCE = 3.5; // meters

        // TODO: verify correctness
        public static final Transform2d CAMERA_TO_ROBOT = new Transform2d(-0.2775, -0.0635, Rotation2d.fromDegrees(-90));
        public static final double X_ROT_LOCK_ERROR = 0.075;
    }

    public class MAIN_SUBSYSTEMS_MANAGER{
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/MainSubsystems/";
    }

    public class CONTROLS{
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Controls/";
    }

    public class RANGE_TABLE{
        public static final double OFFSET_ANGLE = 0.25; // prev. 0.5
    }
}
