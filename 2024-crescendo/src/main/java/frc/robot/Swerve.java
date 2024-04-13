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
import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve {
    private Mk4ModuleConfiguration swerveConfig;
    private NewtonSwerve swerve;
    private ChassisSpeeds lastSpeeds;
    private PIDController snapToController;

    public Swerve(Gyro gyro) {
        lastSpeeds = new ChassisSpeeds();
        
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();

        // drivetrain dimensions
        config.setDriveTrainWidthMeters(SWERVE.DRIVE_TRAIN_WIDTH);
        config.setDriveTrainLengthMeters(SWERVE.DRIVE_TRAIN_LENGTH);
        config.setWheelCircumference(SWERVE.WHEEL_CIRCUMFERENCE);

        // Max Values
        config.setNominalVoltage(POWER.SWERVE_MAX_VOLTAGE);
        config.setMaxVelocityMetersPerSecond(SWERVE.MAX_VELOCITY_METERS_PER_SECOND);

        // // set PID constants
        config.setThrottlePID(SWERVE.THROTTLE_kP, SWERVE.THROTTLE_kI, SWERVE.THROTTLE_kD);
        config.setSteerPID(SWERVE.STEER_kP, SWERVE.STEER_kI, SWERVE.STEER_kD);
        snapToController = new PIDController(SWERVE.SNAP_TO_kP, SWERVE.SNAP_TO_kI, SWERVE.SNAP_TO_kD);

        //TODO: Check the swerve module type and comment/uncomment the next 44 lines to account for it
        SwerveModule m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                CAN.SWERVE_BLACK_FRONT_LEFT_DRIVE_CAN_ID,
                CAN.SWERVE_BLACK_FRONT_LEFT_STEER_CAN_ID,
                CAN.SWERVE_BLACK_FRONT_LEFT_ENCODER_CAN_ID,
                SWERVE.BLACK_FRONT_LEFT_STEER_OFFSET);

        SwerveModule m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                CAN.SWERVE_ORANGE_FRONT_RIGHT_DRIVE_CAN_ID, 
                CAN.SWERVE_ORANGE_FRONT_RIGHT_STEER_CAN_ID,
                CAN.SWERVE_ORANGE_FRONT_RIGHT_ENCODER_CAN_ID, 
                SWERVE.ORANGE_FRONT_RIGHT_STEER_OFFSET);

        SwerveModule m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                CAN.SWERVE_TEAL_BACK_LEFT_DRIVE_CAN_ID, 
                CAN.SWERVE_TEAL_BACK_LEFT_STEER_CAN_ID,
                CAN.SWERVE_TEAL_BACK_LEFT_ENCODER_CAN_ID, 
                SWERVE.TEAL_BACK_LEFT_STEER_OFFSET);

        SwerveModule m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                CAN.SWERVE_WHITE_BACK_RIGHT_DRIVE_CAN_ID, 
                CAN.SWERVE_WHITE_BACK_RIGHT_STEER_CAN_ID,
                CAN.SWERVE_WHITE_BACK_RIGHT_ENCODER_CAN_ID, 
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

    }

    public void drive(ChassisSpeeds speeds) {
        this.lastSpeeds = speeds;
        swerve.drive(speeds);
    }

    public double getYaw() {
        return swerve.getYaw();
    }

    public double getYawRate() {
        return swerve.gyro.getYawRate();
    }

    public double getMaxTranslateVelo() {
        return swerve.getMaxTranslateVelocity();
    }

    public double getMaxAngularVelo() {
        return swerve.getMaxAngularVelocity();
    }

    public Pose2d getCurrentPos() {
        if(Robot.isReal()){
            // return swerve.getCurrentPos();
            return swerve.getCurrentPosVision(); // TODO: TEST!!!
        }
        else{
            return Robot.FIELD.getRobotPose();
        }
    }

    public void addVisionMeasurement(PoseVision vision) {
        swerve.addVisionMeasurement(vision);
    }

    public Rotation2d getGyroscopeRotation() {
        return swerve.getGyroscopeRotation();
    }

    public void setSteerAnglesToAbsEncoder() {
        swerve.resetSteerAngles();
    }

    public void setThrottleCurrentLimit(double limit) {
        swerve.setThrottleCurrentLimit(limit);
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

    public void setGyroscopeRotation(double yaw){
        swerve.gyro.setYaw(yaw);
    }
    public double turnToAngle (double setAngle) {

        double currYaw = getGyroscopeRotation().getRadians();
        double setPoint = setAngle * CONVERSIONS.DEG_TO_RAD;

        double errorAngle = setPoint - currYaw;   
       if(errorAngle > Math.PI){
           errorAngle -= 2*Math.PI;
       } else if(errorAngle <= -Math.PI){
           errorAngle += 2*Math.PI;
       } 

        double out = snapToController.calculate(0, errorAngle);
        

        return out;
    }
}
