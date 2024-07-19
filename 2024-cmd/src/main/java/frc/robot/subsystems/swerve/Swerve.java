// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.NewtonSwerve.*;
import com.NewtonSwerve.Mk4.*;
import com.NewtonSwerve.Gyro.Gyro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.littletonrobotics.junction.Logger;

import frc.robot.*;
import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Swerve extends SubsystemBase {
    public SwerveCommands commands = new SwerveCommands(this);

    private Mk4ModuleConfiguration swerveConfig;
    private NewtonSwerve swerve;
    private PIDController snapToController;

    private boolean isSlowMode;
    private boolean robotOriented;

    // Yaw lock is not currently implemented
    private boolean yawLock;
    private double yawLockValue;

    private SmoothingFilter smoothingFilter;

    /**
     * @param gyro a com.NewtonSwerve.Gyro.Gryo object instantiated with the hardware gyroscope
     */
    public Swerve(Gyro gyro) {
        smoothingFilter = new SmoothingFilter(
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.ROTATION_SMOOTHING_AMOUNT
        );

        // Stores physical information about the swerve and info about user config
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();

        // Drivetrain dimensions
        config.setDriveTrainWidthMeters(SWERVE.DRIVE_TRAIN_WIDTH);
        config.setDriveTrainLengthMeters(SWERVE.DRIVE_TRAIN_LENGTH);
        config.setWheelCircumference(SWERVE.WHEEL_CIRCUMFERENCE);

        // Max Values
        config.setNominalVoltage(POWER.SWERVE_MAX_VOLTAGE);
        config.setMaxVelocityMetersPerSecond(SWERVE.MAX_VELOCITY_METERS_PER_SECOND);

        // Set PID constants
        config.setThrottlePID(SWERVE.THROTTLE_kP, SWERVE.THROTTLE_kI, SWERVE.THROTTLE_kD);
        config.setSteerPID(SWERVE.STEER_kP, SWERVE.STEER_kI, SWERVE.STEER_kD);

        // For the snap-to-angle functionality
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

    public void periodic() {
        Logger.recordOutput(SWERVE.LOG_PATH+"OdometryPosition", getCurrentPos());
    }

    public void simulationPeriodic() {
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain
     *
     * @param speeds the speeds to run the drivetrain at
     */
    protected void drive(ChassisSpeeds speeds){
        swerve.drive(speeds);
    }

    /**
     * Set whether human-input-processed joystick input should be slowed
     *
     * @param slowMode whether to slow the drivetrain
     */
    protected void setSlowMode(boolean slowMode){
        this.isSlowMode = slowMode;
    }

    /**
     * Set whether human-input-processed joystick input should be robot-oriented
     * (as opposed to field-oriented)
     *
     * @param robotOriented whether to run the drivetrain robot-oriented
     */
    protected void setRobotOriented(boolean robotOriented){
        this.robotOriented = robotOriented;
    }

    /**
     * Set the gyroscope heading to 0
     */
    protected void zeroGyroscope(){
        swerve.zeroGyroscope();
    }

    /**
     * Get the current robot yaw in degrees. This method is somewhat
     * deprecated and is replaced by {@link Swerve#getGyroscopeRotation()},
     * which returns a Rotation2d instead.
     */
    public double getYaw() {
        return swerve.getYaw();
    }

    /**
     * Get the current robot yaw rate in degrees per second.
     * Note that this assumes a loop time of 0.02 seconds exactly,
     * so don't depend on it for anything requiring accuracy.
     */
    public double getYawRate() {
        return swerve.gyro.getYawRate();
    }

    public double getMaxTranslateVelocity() {
        return swerve.getMaxTranslateVelocity();
    }

    public double getMaxAngularVelocity() {
        return swerve.getMaxAngularVelocity();
    }

    /**
     * Get the current position of the swerve as judged by odometry. If
     * in simulation, returns the robot's position on the simulated field
     */
    public Pose2d getCurrentPos() {
        if(Robot.isReal()){
            return swerve.getCurrentPos();
        }
        else{
            return Robot.FIELD.getRobotPose();
        }
    }

    /**
     * Get the robot's rotation as a Rotation2d. Preferable
     * to {@link Swerve#getYaw()}.
     */
    public Rotation2d getGyroscopeRotation() {
        return swerve.getGyroscopeRotation();
    }

    /**
     * Set the robot's known rotation.
     *
     * @param yaw the rotation to set in degrees
     */
    public void setGyroscopeRotation(double yaw){
        swerve.gyro.setYaw(yaw);
    }

    /**
     * Reset the swerve modules' known rotations to the readings from the abolute
     * encoders.
     */
    protected void resetToAbsEncoders() {
        swerve.resetSteerAngles();
        Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Swerve steer angles reset.");
    }

    /**
     * Set the throttle motors' current limits.
     *
     * @param limit the limit to set in amps
     */
    protected void setThrottleCurrentLimit(double limit) {
        swerve.setThrottleCurrentLimit(limit);
        Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Swerve throttle current limit set.");
    }

    /**
     * Reset the robot's known position.
     *
     * @param pose the pose to set the robot's known position to.
     */
    protected void resetPose(Pose2d pose) {
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

    /**
     * Reset all throttle encoders to 0.
     */
    protected void resetEncoders() {
        swerve.resetEncoders();
        Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Swerve throttle encoders reset.");
    }

    /**
     * Use PID to snap the robot to the setpoint.
     *
     * @param setpoint the setpoint to snap to
     * @return the rotational velocity setpoint in radians/second
     */
    protected double snapToAngle(Rotation2d setpoint) {
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

    /**
     * Process joystick inputs for human control
     *
     * @param rawX the raw X input from a joystick. Should be -1 to 1
     * @param rawY the raw Y input from a joystick. Should be -1 to 1
     * @param rawRot the raw rotation input from a joystick. Should be -1 to 1
     * @param fieldOrientedAllowed if this is true, switch between field- and
     * robot-oriented based on {@link Swerve#robotOriented}. Otherwise, force
     * robot-oriented.
     *
     * @return a ChassisSpeeds ready to be sent to the swerve.
     */
    protected ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot, boolean fieldOrientedAllowed){
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
                driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * getMaxTranslateVelocity(),
                driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * getMaxTranslateVelocity(),
                driveRotate * SWERVE.ROTATE_POWER_SLOW * getMaxAngularVelocity()
            ));
        }
        else {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * getMaxTranslateVelocity(),
                driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * getMaxTranslateVelocity(),
                driveRotate * SWERVE.ROTATE_POWER_FAST * getMaxAngularVelocity()
            ));
        }

        currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            currentSpeeds, (
                robotOriented || !fieldOrientedAllowed
                ? new Rotation2d()
                : swerve.getGyroscopeRotation()
            )
        );

        return currentSpeeds;
    }
}
