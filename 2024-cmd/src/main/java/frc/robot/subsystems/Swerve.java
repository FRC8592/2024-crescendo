// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import com.NewtonSwerve.*;
import com.NewtonSwerve.Mk4.*;
import com.NewtonSwerve.Gyro.Gyro;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import frc.robot.*;
import frc.robot.helpers.*;

public class Swerve extends SubsystemBase {
    private Mk4ModuleConfiguration swerveConfig;
    private NewtonSwerve swerve;
    private PIDController snapToController;

    private boolean isSlowMode;
    private boolean robotOriented;
    private ChassisSpeeds lastSpeeds;
    private boolean yawLock;
    private double yawLockValue;

    private SmoothingFilter smoothingFilter;

    public Swerve(Gyro gyro) {
        lastSpeeds = new ChassisSpeeds();

        smoothingFilter = new SmoothingFilter(
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.ROTATION_SMOOTHING_AMOUNT
        );

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
            SWERVE.BLACK_FRONT_LEFT_STEER_OFFSET
        );

        SwerveModule m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            CAN.SWERVE_ORANGE_FRONT_RIGHT_DRIVE_CAN_ID,
            CAN.SWERVE_ORANGE_FRONT_RIGHT_STEER_CAN_ID,
            CAN.SWERVE_ORANGE_FRONT_RIGHT_ENCODER_CAN_ID,
            SWERVE.ORANGE_FRONT_RIGHT_STEER_OFFSET
        );

        SwerveModule m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            CAN.SWERVE_TEAL_BACK_LEFT_DRIVE_CAN_ID,
            CAN.SWERVE_TEAL_BACK_LEFT_STEER_CAN_ID,
            CAN.SWERVE_TEAL_BACK_LEFT_ENCODER_CAN_ID,
            SWERVE.TEAL_BACK_LEFT_STEER_OFFSET
        );

        SwerveModule m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            CAN.SWERVE_WHITE_BACK_RIGHT_DRIVE_CAN_ID,
            CAN.SWERVE_WHITE_BACK_RIGHT_STEER_CAN_ID,
            CAN.SWERVE_WHITE_BACK_RIGHT_ENCODER_CAN_ID,
            SWERVE.WHITE_BACK_RIGHT_STEER_OFFSET
        );

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

        this.swerve = new NewtonSwerve(
            config,
            gyro,
            m_frontLeftModule,
            m_frontRightModule,
            m_backLeftModule,
            m_backRightModule
        );

    }
    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command driveCommand(DoubleSupplier suppliedX, DoubleSupplier suppliedY, DoubleSupplier suppliedRot) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(() -> {
            swerve.drive(processJoystickInputs(
                suppliedX.getAsDouble(),
                suppliedY.getAsDouble(),
                suppliedRot.getAsDouble()
            ));
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command snapToCommand(
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        Rotation2d angle){
        return run(() -> {
            ChassisSpeeds processed = processJoystickInputs(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                0
            );

            processed.omegaRadiansPerSecond = snapToAngle(angle);
            swerve.drive(processed);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command noExpoRotationCommand(
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSpeedSupplier){

        return run(() -> {
            ChassisSpeeds processed = processJoystickInputs(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                0
            );

            processed.omegaRadiansPerSecond = rotationSpeedSupplier.getAsDouble();
            swerve.drive(processed);
        });
    }

    private Command chassisSpeedsDriveCommand(ChassisSpeeds speeds){
        return runOnce(() -> {swerve.drive(speeds);});
    }

    // NOTE: We return Commands.runOnce() instead of this.runOnce() on the slowMode,
    // robotOriented, and zeroGyroscope commands. This means the commands don't
    // require this subsystem, which will avoid any problems from the scheduler
    // being fussy about same-subsystem commands running at the same time.
    //
    // PLEASE NOTE that this is usually a horrible idea because of the risk of
    // commands fighting each other. It should be fine in this case because these
    // commands don't move the swerve, but this should generally be done with
    // caution.
    public Command slowModeCommand(boolean slowMode){
        return Commands.runOnce(() -> {
            this.isSlowMode = slowMode;
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", slowMode?"Slow mode enabled":"Slow mode disabled");
        });
    }

    public Command robotOrientedCommand(boolean robotOriented){
        return Commands.runOnce(() -> {
            this.robotOriented = robotOriented;
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", robotOriented?"Robot-oriented enabled":"Robot-oriented disabled");
        });
    }

    public Command zeroGyroscopeCommand() {
        return Commands.runOnce(() -> {
            swerve.zeroGyroscope();
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Gyroscope zeroed");
        });
    }

    public Command autonomousInit(){
        return runOnce(() -> {
            this.resetEncoder();
            this.resetPose(new Pose2d());
            this.resetToAbsEncoders();
            this.setThrottleCurrentLimit(POWER.SWERVE_AUTO_THROTTLE_CURRENT_LIMIT);
        }).alongWith(
            this.chassisSpeedsDriveCommand(new ChassisSpeeds())
        ).alongWith(
            this.zeroGyroscopeCommand()
        );
    }

    public void periodic() {
        Logger.recordOutput(SWERVE.LOG_PATH+"OdometryPosition", getCurrentPos());
    }

    public void simulationPeriodic() {
        Logger.recordOutput(SWERVE.LOG_PATH+"OdometryPosition", getCurrentPos());
        Robot.FIELD.setRobotPose(getCurrentPos());
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
            return swerve.getCurrentPos();
        }
        else{
            return Robot.FIELD.getRobotPose();
        }
    }

    public Rotation2d getGyroscopeRotation() {
        return swerve.getGyroscopeRotation();
    }


    public ChassisSpeeds getCurrentSpeeds() {
        return lastSpeeds;
    }

    public void setGyroscopeRotation(double yaw){
        swerve.gyro.setYaw(yaw);
    }

    private void resetToAbsEncoders() {
        swerve.resetSteerAngles();
        Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Swerve steer angles reset.");
    }

    private void setThrottleCurrentLimit(double limit) {
        swerve.setThrottleCurrentLimit(limit);
        Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Swerve throttle current limit set.");
    }

    private void resetPose(Pose2d pose) {
        swerve.resetPose(pose);
        Logger.recordOutput(
            SWERVE.LOG_PATH+"Console", (
                "Current pose reset to X: "+
                pose.getX()+
                "; Y: "+
                pose.getY()+
                "; Rotation: "+
                pose.getRotation().getDegrees()+
                "°."
            )
        );
    }

    private void resetEncoder() {
        swerve.resetEncoder();
        Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Swerve throttle encoders reset.");
    }

    private double snapToAngle (Rotation2d setpoint) {
        double currYaw = getGyroscopeRotation().getRadians();
        double errorAngle = setpoint.getRadians() - currYaw;

        if(errorAngle > Math.PI){
            errorAngle -= 2*Math.PI;
        }
        else if(errorAngle <= -Math.PI){
            errorAngle += 2*Math.PI;
        }

        double out = snapToController.calculate(0, errorAngle);

        return out;
    }

    private ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot){
        double driveTranslateY = (
            rawY >= 0
            ? (Math.pow(rawY, SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(rawY, SWERVE.JOYSTICK_EXPONENT))
        );

        double driveTranslateX = (
            rawX >= 0 
            ? (Math.pow(rawX, SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(rawX, SWERVE.JOYSTICK_EXPONENT))
        );

        double driveRotate = (
            rawRot >= 0
            ? (Math.pow(rawRot, SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(rawRot, SWERVE.JOYSTICK_EXPONENT))
        );

        //
        // Lock the robot yaw if the rotation rate is low and the yaw joystick is released
        // Only unlock the robot yaw if the joystick provides a yaw command
        //
        // if ((Math.abs(getYawRate()) < 5) && (Math.abs(rawRot) < 0.05)) {
        //     if (!yawLock) {
        //         yawLockValue = getYaw();
        //     }
        //     yawLock = true;
        // }

        // if (Math.abs(rawRot) > 0.05) {
        //     yawLock = false;
        // }

        // Logger.recordOutput(SWERVE.LOG_PATH+"YawLock", yawLock);
        // Logger.recordOutput(SWERVE.LOG_PATH+"YawLast", yawLockValue);
        // Logger.recordOutput(SWERVE.LOG_PATH+"YawRate", getYawRate());
        // Logger.recordOutput(SWERVE.LOG_PATH+"rawRot", rawRot);

        //Create a new ChassisSpeeds object with X, Y, and angular velocity from controller input
        ChassisSpeeds currentSpeeds;

        if (isSlowMode) { //Slow Mode slows down the robot for better precision & control
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * getMaxTranslateVelo(),
                driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * getMaxTranslateVelo(),
                driveRotate * SWERVE.ROTATE_POWER_SLOW * getMaxAngularVelo()
            ));
        }
        else {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * getMaxTranslateVelo(),
                driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * getMaxTranslateVelo(),
                driveRotate * SWERVE.ROTATE_POWER_FAST * getMaxAngularVelo()
            ));
        }

        currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            currentSpeeds,
            (
                robotOriented 
                ? new Rotation2d() 
                : swerve.getGyroscopeRotation()
            )
        );

        return currentSpeeds;
    }
}
