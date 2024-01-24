package frc.robot;

import com.NewtonSwerve.NewtonSwerve;
import com.NewtonSwerve.SwerveModule;
import com.NewtonSwerve.Gyro.NewtonPigeon;
import com.NewtonSwerve.Mk4.Mk4ModuleConfiguration;
import com.NewtonSwerve.Mk4.Mk4SwerveModuleHelper;
import com.NewtonSwerve.Mk4.Mk4iSwerveModuleHelper;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Swerve {


    public static final int PIGEON_CAN_ID = -1; // TODO: Value isn't set.

    public static final double THROTTLE_kP = 0.02;
    public static final double THROTTLE_kI = 0.0;
    public static final double THROTTLE_kD = 0.01;

    public static final double STEER_kP = 0.2;
    public static final double STEER_kI = 0.0;
    public static final double STEER_kD = 0.1;

    public static final double DRIVE_TRAIN_WIDTH = -1; // meters // TODO: Value isn't set.
    public static final double DRIVE_TRAIN_LENGTH = -1; // meters // TODO: Value isn't set.
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI; // TODO: Check that this is accurate

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-1);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-1); // TODO: Value isn't set.
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-1);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-1);

    public static final double TRANSLATE_POWER_FAST = 1.0; // Scaling for teleop driving. 1.0 is maximum
    public static final double ROTATE_POWER_FAST = 0.75; // Scaling for teleop driving. 1.0 is maximum
    public static final double TRANSLATE_POWER_SLOW = 0.15; // Scaling for teleop driving. 1.0 is maximum
    public static final double ROTATE_POWER_SLOW = 0.15; // Scaling for teleop driving. 1.0 is maximum

    public static final double MAX_VOLTAGE = 12.0;
    public static final double TELEOP_CURRENT_LIMIT = -1; // TODO: Value isn't set.
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;

    private Mk4ModuleConfiguration swerveConfig;
    private NewtonSwerve swerve;

    public Swerve(NewtonPigeon gyro) {
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();

        // drivetrain dimensions
        config.setDriveTrainWidthMeters(DRIVE_TRAIN_WIDTH);
        config.setDriveTrainLengthMeters(DRIVE_TRAIN_LENGTH);
        config.setWheelCircumference(WHEEL_CIRCUMFERENCE);

        // Max Values
        config.setNominalVoltage(MAX_VOLTAGE);
        config.setMaxVelocityMetersPerSecond(MAX_VELOCITY_METERS_PER_SECOND);
        config.setTelelopCurrentLimit(TELEOP_CURRENT_LIMIT);

        // // set PID constants
        config.setThrottlePID(THROTTLE_kP, THROTTLE_kI, THROTTLE_kD);
        config.setSteerPID(STEER_kP, STEER_kI, STEER_kD);

        //TODO: Check the swerve module type and comment/uncomment the next 44 lines to account for it
        // SwerveModule m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
        //         Mk4iSwerveModuleHelper.GearRatio.L2,
        //         FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
        //         FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID,
        //         FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID,
        //         FRONT_LEFT_MODULE_STEER_OFFSET);

        // SwerveModule m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
        //         Mk4iSwerveModuleHelper.GearRatio.L2,
        //         FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID, FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID,
        //         FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID, FRONT_RIGHT_MODULE_STEER_OFFSET);

        // SwerveModule m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
        //         Mk4iSwerveModuleHelper.GearRatio.L2,
        //         BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID, BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID,
        //         BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID, BACK_LEFT_MODULE_STEER_OFFSET);

        // SwerveModule m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
        //         Mk4iSwerveModuleHelper.GearRatio.L2,
        //         BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID, BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID,
        //         BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID, BACK_RIGHT_MODULE_STEER_OFFSET);

        SwerveModule m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(config,
                Mk4SwerveModuleHelper.GearRatio.L2, Hardware.FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
                Hardware.FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID,
                Hardware.FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        SwerveModule m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(config,
                Mk4SwerveModuleHelper.GearRatio.L2, Hardware.FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
                Hardware.FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID,
                Hardware.FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        SwerveModule m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(config,
                Mk4SwerveModuleHelper.GearRatio.L2, Hardware.BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
                Hardware.BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID,
                Hardware.BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID, BACK_LEFT_MODULE_STEER_OFFSET);

        SwerveModule m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(config,
                Mk4SwerveModuleHelper.GearRatio.L2, Hardware.BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
                Hardware.BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID,
                Hardware.BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        this.swerveConfig = config;

        this.swerve = new NewtonSwerve(config, gyro, m_frontLeftModule, m_frontRightModule, m_backLeftModule,
                m_backRightModule);

    }

    public void drive(ChassisSpeeds speeds) {
        swerve.drive(speeds);
    }

    public double getYaw() {
        return swerve.getYaw();
    }

    public double getMaxTranslateVelo() {
        return swerve.getMaxTranslateVelocity();
    }

    public double getMaxAngularVelo() {
        return swerve.getMaxAngularVelocity();
    }

    public Pose2d getCurrentPos() {
        return swerve.getCurrentPos();
    }

    public Rotation2d getGyroscopeRotation() {
        return swerve.getGyroscopeRotation();
    }

    public void setSteerAnglesToAbsEncoder() {
        swerve.resetSteerAngles();
    }

    public void setTeleopCurrentLimit() {
        swerve.setTeleopCurrentLimit();
    }

    public void setAutoCurrentLimit() {
        swerve.setAutoCurrentLimit();
    }

    public void resetPose(Pose2d pose) {
        swerve.resetPose(pose);
    }

    public void resetEncoder() {
        swerve.resetEncoder();
    }
}
