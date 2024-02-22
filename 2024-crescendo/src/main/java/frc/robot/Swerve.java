package frc.robot;

import com.NewtonSwerve.NewtonSwerve;
import com.NewtonSwerve.SwerveModule;
// import com.NewtonSwerve.Gyro.NewtonPigeon2;
import com.NewtonSwerve.Gyro.Gyro;
import com.NewtonSwerve.Gyro.NewtonPigeon;
import com.NewtonSwerve.Mk4.Mk4ModuleConfiguration;
import com.NewtonSwerve.Mk4.Mk4SwerveModuleHelper;
import com.NewtonSwerve.Mk4.Mk4iSwerveModuleHelper;
import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Swerve {
    private Mk4ModuleConfiguration swerveConfig;
    private NewtonSwerve swerve;
    private ChassisSpeeds lastSpeeds;
    private PIDController turnPidController;

    public Swerve(Gyro gyro) {
        lastSpeeds = new ChassisSpeeds();
        
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();

        // drivetrain dimensions
        config.setDriveTrainWidthMeters(SWERVE.DRIVE_TRAIN_WIDTH);
        config.setDriveTrainLengthMeters(SWERVE.DRIVE_TRAIN_LENGTH);
        config.setWheelCircumference(SWERVE.WHEEL_CIRCUMFERENCE);

        // Max Values
        config.setNominalVoltage(SWERVE.MAX_VOLTAGE);
        config.setMaxVelocityMetersPerSecond(SWERVE.MAX_VELOCITY_METERS_PER_SECOND);
        config.setTelelopCurrentLimit(SWERVE.TELEOP_CURRENT_LIMIT);

        // // set PID constants
        config.setThrottlePID(SWERVE.THROTTLE_kP, SWERVE.THROTTLE_kI, SWERVE.THROTTLE_kD);
        config.setSteerPID(SWERVE.STEER_kP, SWERVE.STEER_kI, SWERVE.STEER_kD);

        //TODO: Check the swerve module type and comment/uncomment the next 44 lines to account for it
        SwerveModule m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                SWERVE.BLACK_FRONT_LEFT_DRIVE_CAN,
                SWERVE.BLACK_FRONT_LEFT_STEER_CAN,
                SWERVE.BLACK_FRONT_LEFT_ENCODER_CAN,
                SWERVE.BLACK_FRONT_LEFT_STEER_OFFSET);

        SwerveModule m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                SWERVE.ORANGE_FRONT_RIGHT_DRIVE_CAN, 
                SWERVE.ORANGE_FRONT_RIGHT_STEER_CAN,
                SWERVE.ORANGE_FRONT_RIGHT_ENCODER_CAN, 
                SWERVE.ORANGE_FRONT_RIGHT_STEER_OFFSET);

        SwerveModule m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                SWERVE.TEAL_BACK_LEFT_DRIVE_CAN, 
                SWERVE.TEAL_BACK_LEFT_STEER_CAN,
                SWERVE.TEAL_BACK_LEFT_ENCODER_CAN, 
                SWERVE.TEAL_BACK_LEFT_STEER_OFFSET);

        SwerveModule m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                SWERVE.WHITE_BACK_RIGHT_DRIVE_CAN, 
                SWERVE.WHITE_BACK_RIGHT_STEER_CAN,
                SWERVE.WHITE_BACK_RIGHT_ENCODER_CAN, 
                SWERVE.WHITE_BACK_RIGHT_STEER_OFFSET);

        // SwerveModule m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(config,
        //         Mk4SwerveModuleHelper.GearRatio.L2, SWERVE.BLACK_FRONT_LEFT_DRIVE_CAN,
        //         SWERVE.BLACK_FRONT_LEFT_STEER_CAN,
        //         SWERVE.BLACK_FRONT_LEFT_ENCODER_CAN,
        //         SWERVE.BLACK_FRONT_LEFT_STEER_OFFSET);
        // SwerveModule m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(config,
        //         Mk4SwerveModuleHelper.GearRatio.L2, SWERVE.TEAL_FRONT_LEFT_DRIVE_CAN,
        //         SWERVE.TEAL_FRONT_LEFT_STEER_CAN,
        //         SWERVE.TEAL_FRONT_LEFT_ENCODER_CAN,
        //         SWERVE.FRONT_LEFT_STEER_OFFSET);

        // SwerveModule m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(config,
        //         Mk4SwerveModuleHelper.GearRatio.L2, SWERVE.ORANGE_FRONT_RIGHT_DRIVE_CAN,
        //         SWERVE.ORANGE_FRONT_RIGHT_STEER_CAN,
        //         SWERVE.ORANGE_FRONT_RIGHT_ENCODER_CAN,
        //         SWERVE.FRONT_RIGHT_STEER_OFFSET);

        // SwerveModule m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(config,
        //         Mk4SwerveModuleHelper.GearRatio.L2, SWERVE.BLACK_BACK_LEFT_DRIVE_CAN,
        //         SWERVE.BLACK_BACK_LEFT_STEER_CAN,
        //         SWERVE.BLACK_BACK_LEFT_ENCODER_CAN,
        //         SWERVE.BACK_LEFT_STEER_OFFSET);

        // SwerveModule m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(config,
        //         Mk4SwerveModuleHelper.GearRatio.L2, SWERVE.WHITE_BACK_RIGHT_DRIVE_CAN,
        //         SWERVE.WHITE_BACK_RIGHT_STEER_CAN,
        //         SWERVE.WHITE_BACK_RIGHT_ENCODER_CAN,
        //         SWERVE.BACK_RIGHT_STEER_OFFSET);

        this.swerveConfig = config;

        this.swerve = new NewtonSwerve(config, gyro, m_frontLeftModule, m_frontRightModule, m_backLeftModule,
                m_backRightModule);

        turnPidController = new PIDController(Constants.SWERVE.TURN_kP, Constants.SWERVE.TURN_kI, Constants.SWERVE.TURN_kD);
    }

    public void drive(ChassisSpeeds speeds) {
        this.lastSpeeds = speeds;
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

    public ChassisSpeeds getCurrentSpeeds() {
        return lastSpeeds;
    }
    
    public void zeroGyroscope() {
        swerve.zeroGyroscope();
    }

    public double turnToAngle(double angle) {
        return turnPidController.calculate(swerve.getYaw(), angle);

    }
}
