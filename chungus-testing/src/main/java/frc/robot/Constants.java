package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    private Constants() {throw new UnsupportedOperationException();}




    /****************************************************************************************
     * Conversions
    *****************************************************************************************/
    public static final double  CONVERSION_METERS_SECOND_TO_TICKS = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));
    public static final double  CONVERSION_RPM_TO_TICKS_100_MS = 2048.0 / 600.0;     // Conversion factor for rotational velocity (RPM to ticks per 100ms)




    /****************************************************************************************
     * Measurements
    *****************************************************************************************/
    public static final double  MEASUREMENT_TICKS_PER_REVOLUTION = 2048;
    public static final double  MEASUREMENT_FIELD_LONG_SIDE_METERS = 16.4592; //These constants are just there to as a
    public static final double  MEASUREMENT_FIELD_SHORT_SIDE_METERS = 8.2296; //reminder and to make the code more readable.




    /****************************************************************************************
     * Controllers
    *****************************************************************************************/
    public static final double  CONTROLLER_JOYSTICK_DEADBAND = 0.01;
    public static final int     CONTROLLER_DRIVER_PORT = 0;
    public static final int     CONTROLLER_OPERATOR_PORT = 1;




   /****************************************************************************************
     * Intake
    *****************************************************************************************/
    public static final int     INTAKE_ROLLER_MOTOR_CAN_ID = 15; 
    public static final int     INTAKE_PNEUMATICS_CAN_ID = 20; 
    public static final int     INTAKE_SOLENOID_PORT_A = 8; 
    public static final int     INTAKE_SOLENOID_PORT_B = 9;

    public static final double  INTAKE_ROLLER_MOTOR_kP = 0.0003;

    public static final double  INTAKE_ROLLER_SPEED = 0.7;
    public static final double  INTAKE_GEAR_RATIO = 15.0/34.0;
    private static final double INTAKE_DIAMETER_INCHES = 1.67;
    private static final double INTAKE_CIRCUMFERENCE_METERS = INTAKE_DIAMETER_INCHES * 0.0254 * Math.PI;
    public static final double  INTAKE_REVOLUTIONS_PER_METER = 1.0 / INTAKE_CIRCUMFERENCE_METERS;
    public static final double  INTAKE_ADJUST_ROLLER_TICKS_SCALE = 1.05;

    // private static final double MOTOR_REVOLUTIONS_PER_METER = INTAKE_CIRCUMFERENCE_METERS * INTAKE_GEAR_RATIO;




    /****************************************************************************************
     * Dead Axle Pop (Shooter)
    *****************************************************************************************/
    public static final int     DAP_BACK_MOTOR_CAN_ID = 18;
    public static final int     DAP_FRONT_MOTOR_CAN_ID = 19;

    public static final double  DAP_FRONT_MOTOR_kP = 0.1;
    public static final double  DAP_FRONT_MOTOR_kI = 0.0;
    public static final double  DAP_FRONT_MOTOR_kD = 0.0;
    public static final double  DAP_FRONT_MOTOR_kF = 0.55;
    
    public static final double  DAP_BACK_MOTOR_kP = 0.05;
    public static final double  DAP_BACK_MOTOR_kI = 0.0;
    public static final double  DAP_BACK_MOTOR_kD = 0.0;
    public static final double  DAP_BACK_MOTOR_kF = 0.05;

    public static final double  FRONT_SHOOTER_SPEED_TICKS_100MS = 19000;
    public static final double  BACK_SHOOTER_SPEED_TICKS_100MS = 5800;

    public static final double  DAP_FRONT_VELOCITY_TARGET= 2000;
    public static final double  DAP_BOTTOM_DELOCITY_TARGET = 2000;
    public static final double  DAP_VELOCITY_TOLERANCE = 100;
    public static final double  DAP_INTAKE_MOVEMENT_POWER_FOR_SHOOT = 1.0;

    public static final double  CRESCENDO_SHOOTER_kP =  0.000035;
    public static final double  CRESCENDO_SHOOTER_kI =  0.0;
    public static final double  CRESCENDO_SHOOTER_kD =  0.0;
    public static final double  CRESCENDO_SHOOTER_kFF = 0.0001535;


    /****************************************************************************************
     * Bunny Dropper
    *****************************************************************************************/
    public static final int     BUNNY_DROPPER_CANID = 23;

    public static final double  BUNNY_DROPPER_kP = 0.3;
    public static final double  BUNNY_DROPPER_kI = 0.0;
    public static final double  BUNNY_DROPPER_kD = 0.01;

    public static final int     BUNNY_DROPPER_GEAR_RATIO = 16;
    public static final double  BUNNY_DROPPER_DROP_POSITION = 0.5 * BUNNY_DROPPER_GEAR_RATIO;
    public static final double  BUNNY_DROPPER_MOTOR_SPEED = 0.3;
    public static final int     BUNNY_DROPPER_ACCEPTABLE_DISTANCE_TICKS = 2;
    public static final double  BUNNY_DROPPER_HOLD_POSITION = 0.0;




    /****************************************************************************************
     * Swerve (Drivetrain)
     * 
     * TEAL is front-left
     * ORANGE is front-right
     * BLACK is back-right
     * WHITE is back-left
    *****************************************************************************************/
    public static final int     SWERVE_FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID = 7;
    public static final int     SWERVE_FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID = 6;
    public static final int     SWERVE_FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID = 8;

    public static final int     SWERVE_FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID = 4;
    public static final int     SWERVE_FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID = 3;
    public static final int     SWERVE_FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID = 5;

    public static final int     SWERVE_BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID = 10;
    public static final int     SWERVE_BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID = 9;
    public static final int     SWERVE_BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID = 11;

    public static final int     SWERVE_BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID = 13;
    public static final int     SWERVE_BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID = 12;
    public static final int     SWERVE_BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID = 14;

    public static final int     SWERVE_PIGEON_CAN_ID = 17;


    public static final double  SWERVE_THROTTLE_kP = 0.02;
    public static final double  SWERVE_THROTTLE_kI = 0.0;
    public static final double  SWERVE_THROTTLE_kD = 0.01;

    public static final double  SWERVE_STEER_kP = 0.2;
    public static final double  SWERVE_STEER_kI = 0.0;
    public static final double  SWERVE_STEER_kD = 0.1;


    public static final double  SWERVE_DRIVE_TRAIN_WIDTH = 0.498; // meters
    public static final double  SWERVE_DRIVE_TRAIN_LENGTH = 0.52; // meters
    public static final double  SWERVE_WHEEL_CIRCUMFERENCE = 4 * Math.PI;

    // We put the offsets in the Magnet Offset in Phoenix Tuner so that we don't need to hardcode them here anymore
    public static final double  SWERVE_FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(233.525);
    public static final double  SWERVE_FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(133.77);
    public static final double  SWERVE_BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(78.75);
    public static final double  SWERVE_BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(147.041);

    public static final double  SWERVE_TRANSLATE_POWER_FAST = 1.0;      // Scaling for teleop driving.  1.0 is maximum
    public static final double  SWERVE_ROTATE_POWER_FAST    = 0.75;      // Scaling for teleop driving.  1.0 is maximum
    public static final double  SWERVE_TRANSLATE_POWER_SLOW = 0.15;     // Scaling for teleop driving.  1.0 is maximum
    public static final double  SWERVE_ROTATE_POWER_SLOW    = 0.15;     // Scaling for teleop driving.  1.0 is maximum   

    public static final double  SWERVE_MAX_VOLTAGE = 12.0;
    public static final double  SWERVE_MAX_VELOCITY_METERS_PER_SECOND = 4.5;

    public static final double TELEOP_CURRENT_LIMIT = 15.0;
    
    public static final double MOVE_BACK_AIM_TIME = 0.4;
    public static final double MOVE_BACK_AIM_SPEED = 0.6;

    public static final double SPARK_FLEX_MAX_VELOCITY = 6784;



    /****************************************************************************************
     * Robot and field element locations
    *****************************************************************************************/
    public static final Pose2d  LOCATION_YARD_POSITION = new Pose2d(MEASUREMENT_FIELD_LONG_SIDE_METERS/2, MEASUREMENT_FIELD_SHORT_SIDE_METERS/2, new Rotation2d(0));
    public static final Pose2d  LOCATION_BURROW = new Pose2d(0, MEASUREMENT_FIELD_SHORT_SIDE_METERS, new Rotation2d(0));
    public static final Pose2d  LOCATION_OUTSIDE_DEN = new Pose2d(1, 1.3368, new Rotation2d(0));
    public static final Pose2d  LOCATION_AVOID_BARRIER = new Pose2d(MEASUREMENT_FIELD_LONG_SIDE_METERS/4 - 0.5, 1, new Rotation2d(0));
    public static final Pose2d  LOCATION_START = new Pose2d(0,2.3368,new Rotation2d(0));

    public static final Pose2d  LOCATION_BUNNY_DROPOFF = LOCATION_AVOID_BARRIER.transformBy(new Transform2d(
        new Translation2d(1.75, 1.75),
        new Rotation2d(0))
    );
    public static final Pose2d  LOCATION_BERRY_DROPOFF = LOCATION_BURROW.transformBy(new Transform2d(
        new Translation2d(1, -1),
        new Rotation2d(-135))
    );
}
