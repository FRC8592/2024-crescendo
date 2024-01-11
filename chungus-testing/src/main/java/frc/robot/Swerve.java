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
    private Mk4ModuleConfiguration swerveConfig;
    private NewtonSwerve swerve;

    public Swerve(NewtonPigeon gyro){
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
        
        // drivetrain dimensions
        config.setDriveTrainWidthMeters(Constants.SWERVE_DRIVE_TRAIN_WIDTH);
        config.setDriveTrainLengthMeters(Constants.SWERVE_DRIVE_TRAIN_LENGTH);
        config.setWheelCircumference(Constants.SWERVE_WHEEL_CIRCUMFERENCE);
        
        // Max Values
        config.setNominalVoltage(Constants.SWERVE_MAX_VOLTAGE);
        config.setMaxVelocityMetersPerSecond(Constants.SWERVE_MAX_VELOCITY_METERS_PER_SECOND);
        config.setTelelopCurrentLimit(Constants.TELEOP_CURRENT_LIMIT);
        
        // // set PID constants
        config.setThrottlePID(Constants.SWERVE_THROTTLE_kP,Constants.SWERVE_THROTTLE_kI, Constants.SWERVE_THROTTLE_kD); // 0.02, 0, 0.01
        config.setSteerPID(Constants.SWERVE_STEER_kP, Constants.SWERVE_STEER_kI, Constants.SWERVE_STEER_kD);
        
        // SwerveModule m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config, Mk4iSwerveModuleHelper.GearRatio.L2, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID, Constants.FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID, Constants.FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID, Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
        
        // SwerveModule m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(config, Mk4iSwerveModuleHelper.GearRatio.L2, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,Constants.FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID, Constants.FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID,Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);
        
        // SwerveModule m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config, Mk4iSwerveModuleHelper.GearRatio.L2, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,Constants.BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID, Constants.BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID,Constants.BACK_LEFT_MODULE_STEER_OFFSET);
        
        // SwerveModule m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(config, Mk4iSwerveModuleHelper.GearRatio.L2, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,Constants.BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID, Constants.BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID,Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
        
        // FOR FUTURE BUNNY BOT WITH MK4 MODULES
        
        SwerveModule m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(config, Mk4SwerveModuleHelper.GearRatio.L2, Constants.SWERVE_FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID, Constants.SWERVE_FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID, Constants.SWERVE_FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID, Constants.SWERVE_FRONT_LEFT_MODULE_STEER_OFFSET);
        
        SwerveModule m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(config, Mk4SwerveModuleHelper.GearRatio.L2, Constants.SWERVE_FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,Constants.SWERVE_FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID, Constants.SWERVE_FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID,Constants.SWERVE_FRONT_RIGHT_MODULE_STEER_OFFSET);
        
        SwerveModule m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(config, Mk4SwerveModuleHelper.GearRatio.L2, Constants.SWERVE_BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,Constants.SWERVE_BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID, Constants.SWERVE_BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID,Constants.SWERVE_BACK_LEFT_MODULE_STEER_OFFSET);
        
        SwerveModule m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(config, Mk4SwerveModuleHelper.GearRatio.L2, Constants.SWERVE_BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,Constants.SWERVE_BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID, Constants.SWERVE_BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID,Constants.SWERVE_BACK_RIGHT_MODULE_STEER_OFFSET);
        
        this.swerveConfig = config;

        this.swerve = new NewtonSwerve(config, gyro, m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule);
        
    }

    public void drive(ChassisSpeeds speeds){
        swerve.drive(speeds);
    }

    public double getYaw(){
        return swerve.getYaw();
    }

    public double getMaxTranslateVelo(){
        return swerve.getMaxTranslateVelocity();
    }

    public double getMaxAngularVelo(){
        return swerve.getMaxAngularVelocity();
    }
    public Pose2d getCurrentPos(){
        return swerve.getCurrentPos();
    }

    public Rotation2d getGyroscopeRotation(){
        return swerve.getGyroscopeRotation();
    }

    public void setSteerAnglesToAbsEncoder(){
        swerve.resetSteerAngles();
    }

    public void setTeleopCurrentLimit(){
        swerve.setTeleopCurrentLimit();
    }
    
    public void setAutoCurrentLimit(){
        swerve.setAutoCurrentLimit();
    }

    public void resetPose(Pose2d pose){
        swerve.resetPose(pose);
    }
    public void resetEncoder(){
        swerve.resetEncoder();
    }
}


