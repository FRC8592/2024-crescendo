// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

import frc.robot.*;
import frc.robot.helpers.*;
import frc.robot.subsystems.NewtonSubsystem;
import frc.robot.Constants.*;

public class Swerve extends NewtonSubsystem {
    private static Swerve instance = null;
    public static Swerve getInstance(){
        if(instance == null){
            throw new IllegalStateException("The Swerve subsystem must be instantiated before attempting to use it");
        }
        return instance;
    }
    public static Swerve instantiate(){
        if(instance != null){
            throw new IllegalStateException("The Swerve subsystem can't be instantiated twice");
        }
        instance = new Swerve();
        return instance;
    }

    public SwerveCommands commands = new SwerveCommands(this);

    /**
     * Small enum to control whether to drive robot- or field-
     * relative for {@link Swerve#drive(ChassisSpeeds, DriveModes)}
     */
    public enum DriveModes{
        /** Drive robot-relative */
        ROBOT_RELATIVE,
        /** Switch between robot- and field-relative depending on driver input */
        AUTOMATIC,
        /** Drive field-relative */
        FIELD_RELATIVE
    }

    private PIDController snapToController;

    private boolean isSlowMode;
    private boolean robotRelative;

    private SmoothingFilter smoothingFilter;

    private CTRESwerve swerve;

    private Swerve() {
        smoothingFilter = new SmoothingFilter(
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.ROTATION_SMOOTHING_AMOUNT
        );

        snapToController = new PIDController(SWERVE.SNAP_TO_kP, SWERVE.SNAP_TO_kI, SWERVE.SNAP_TO_kD);

        // PID constants for the swerve's drive and steer controllers
        Slot0Configs driveGains = (
            new Slot0Configs()
            .withKP(SWERVE.DRIVE_P).withKI(SWERVE.DRIVE_I).withKD(SWERVE.DRIVE_D)
            .withKS(SWERVE.DRIVE_S).withKV(SWERVE.DRIVE_V).withKA(SWERVE.DRIVE_A)
        );
        Slot0Configs steerGains = (
            new Slot0Configs()
            .withKP(SWERVE.STEER_P).withKI(SWERVE.STEER_I).withKD(SWERVE.STEER_D)
            .withKS(SWERVE.STEER_S).withKV(SWERVE.STEER_V).withKA(SWERVE.STEER_A)
        );

        // Drivetrain configuration that doesn't involve the modules
        SwerveDrivetrainConstants drivetrainConstants = (
            new SwerveDrivetrainConstants()
            .withPigeon2Id(CAN.PIGEON_CAN_ID)
            .withPigeon2Configs(new Pigeon2Configuration())
        );

        // This configuration object will apply to all of the swerve's drive motors
        TalonFXConfiguration driveMotorsConfig = (
            new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(POWER.SWERVE_DRIVE_CURRENT_LIMIT)
            )
        );

        // This configuration object will apply to all of the swerve's steer motors
        TalonFXConfiguration steerMotorsConfig = (
            new TalonFXConfiguration().withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(POWER.SWERVE_STEER_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
            )
        );

        // This configuration object will apply to all of the swerve's CANCoders
        CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

        // All swerve module configuration that isn't module-specific
        SwerveModuleConstantsFactory commonSwerveConstants = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(SWERVE.DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(SWERVE.STEER_GEAR_RATIO)
                .withWheelRadius(SWERVE.WHEEL_RADIUS_INCHES)
                .withSlipCurrent(SWERVE.CALCULATED_SLIP_CURRENT)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSpeedAt12VoltsMps(SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND)
                .withDriveInertia(SWERVE.SIMULATED_DRIVE_INERTIA)
                .withSteerInertia(SWERVE.SIMULATED_STEER_INERTIA)
                .withDriveFrictionVoltage(SWERVE.DRIVE_FRICTION_VOLTAGE)
                .withSteerFrictionVoltage(SWERVE.STEER_FRICTION_VOLTAGE)
                .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
                .withCouplingGearRatio(SWERVE.COUPLING_GEAR_RATIO)
                .withDriveMotorInitialConfigs(driveMotorsConfig)
                .withSteerMotorInitialConfigs(steerMotorsConfig)
                .withCANcoderInitialConfigs(cancoderInitialConfigs);

        // Generate swerve-module constant objects by combing the common constants with module-specific ones
        SwerveModuleConstants frontLeft = commonSwerveConstants.createModuleConstants(
            CAN.SWERVE_BLACK_FRONT_LEFT_STEER_CAN_ID,
            CAN.SWERVE_BLACK_FRONT_LEFT_DRIVE_CAN_ID,
            CAN.SWERVE_BLACK_FRONT_LEFT_ENCODER_CAN_ID,
            SWERVE.BLACK_FRONT_LEFT_STEER_OFFSET,
            Units.inchesToMeters(SWERVE.BLACK_FRONT_LEFT_X_POSITION),
            Units.inchesToMeters(SWERVE.BLACK_FRONT_LEFT_Y_POSITION),
            SWERVE.INVERT_LEFT_SIDE
        ).withSteerMotorInverted(SWERVE.BLACK_FRONT_LEFT_STEER_INVERT);
        SwerveModuleConstants frontRight = commonSwerveConstants.createModuleConstants(
            CAN.SWERVE_ORANGE_FRONT_RIGHT_STEER_CAN_ID,
            CAN.SWERVE_ORANGE_FRONT_RIGHT_DRIVE_CAN_ID,
            CAN.SWERVE_ORANGE_FRONT_RIGHT_ENCODER_CAN_ID,
            SWERVE.ORANGE_FRONT_RIGHT_STEER_OFFSET,
            Units.inchesToMeters(SWERVE.ORANGE_FRONT_RIGHT_X_POSITION),
            Units.inchesToMeters(SWERVE.ORANGE_FRONT_RIGHT_Y_POSITION),
            SWERVE.INVERT_RIGHT_SIDE
        ).withSteerMotorInverted(SWERVE.ORANGE_FRONT_RIGHT_STEER_INVERT);
        SwerveModuleConstants backLeft = commonSwerveConstants.createModuleConstants(
            CAN.SWERVE_TEAL_BACK_LEFT_STEER_CAN_ID,
            CAN.SWERVE_TEAL_BACK_LEFT_DRIVE_CAN_ID,
            CAN.SWERVE_TEAL_BACK_LEFT_ENCODER_CAN_ID,
            SWERVE.TEAL_BACK_LEFT_STEER_OFFSET,
            Units.inchesToMeters(SWERVE.TEAL_BACK_LEFT_X_POSITION),
            Units.inchesToMeters(SWERVE.TEAL_BACK_LEFT_Y_POSITION),
            SWERVE.INVERT_LEFT_SIDE
        ).withSteerMotorInverted(SWERVE.TEAL_BACK_LEFT_STEER_INVERT);
        SwerveModuleConstants backRight = commonSwerveConstants.createModuleConstants(
            CAN.SWERVE_WHITE_BACK_RIGHT_STEER_CAN_ID,
            CAN.SWERVE_WHITE_BACK_RIGHT_DRIVE_CAN_ID,
            CAN.SWERVE_WHITE_BACK_RIGHT_ENCODER_CAN_ID,
            SWERVE.WHITE_BACK_RIGHT_STEER_OFFSET,
            Units.inchesToMeters(SWERVE.WHITE_BACK_RIGHT_X_POSITION),
            Units.inchesToMeters(SWERVE.WHITE_BACK_RIGHT_Y_POSITION),
            SWERVE.INVERT_RIGHT_SIDE
        ).withSteerMotorInverted(SWERVE.WHITE_BACK_RIGHT_STEER_INVERT);

        swerve = new CTRESwerve(drivetrainConstants, commonSwerveConstants, frontLeft, frontRight, backLeft, backRight);

        // This lambda is run every time the odometry is updated (100hz for classic CAN or 250hz for CAN FD)
        swerve.registerTelemetry((drivetrainState, kinematics, modules) -> {
            // Logger.recordOutput(SWERVE.LOG_PATH+"TargetSwerveStates", drivetrainState.ModuleTargets);
            // Logger.recordOutput(SWERVE.LOG_PATH+"ReadSwerveStates", drivetrainState.ModuleStates);
            // Logger.recordOutput(SWERVE.LOG_PATH+"OdometryPosition", drivetrainState.Pose);
            // Logger.recordOutput(SWERVE.LOG_PATH+"ActualChassisSpeeds", drivetrainState.speeds);
            // Logger.recordOutput(SWERVE.LOG_PATH+"TargetChassisSpeeds", kinematics.toChassisSpeeds(
            //     modules[0].getTargetState(), modules[1].getTargetState(), modules[2].getTargetState(), modules[3].getTargetState()
            // ));

            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontLeft/DriveReadVelocityMPS", modules[0].getCurrentState().speedMetersPerSecond);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontLeft/DriveTargetVelocityMPS", modules[0].getTargetState().speedMetersPerSecond);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontLeft/SteerReadAngle", modules[0].getCurrentState().angle);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontLeft/SteerTargetAngle", modules[0].getTargetState().angle);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontLeft/SteerReadVelocityRPM", modules[0].getSteerMotor().getVelocity().getValueAsDouble());

            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontRight/DriveReadVelocityMPS", modules[1].getCurrentState().speedMetersPerSecond);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontRight/DriveTargetVelocityMPS", modules[1].getTargetState().speedMetersPerSecond);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontRight/SteerReadAngle", modules[1].getCurrentState().angle);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontRight/SteerTargetAngle", modules[1].getTargetState().angle);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/FrontRight/SteerReadVelocityRPM", modules[1].getSteerMotor().getVelocity().getValueAsDouble());

            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackLeft/DriveReadVelocityMPS", modules[2].getCurrentState().speedMetersPerSecond);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackLeft/DriveTargetVelocityMPS", modules[2].getTargetState().speedMetersPerSecond);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackLeft/SteerReadAngle", modules[2].getCurrentState().angle);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackLeft/SteerTargetAngle", modules[2].getTargetState().angle);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackLeft/SteerReadVelocityRPM", modules[2].getSteerMotor().getVelocity().getValueAsDouble());

            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackRight/DriveReadVelocityMPS", modules[3].getCurrentState().speedMetersPerSecond);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackRight/DriveTargetVelocityMPS", modules[3].getTargetState().speedMetersPerSecond);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackRight/SteerReadAngle", modules[3].getCurrentState().angle);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackRight/SteerTargetAngle", modules[3].getTargetState().angle);
            // Logger.recordOutput(SWERVE.LOG_PATH+"Modules/BackRight/SteerReadVelocityRPM", modules[3].getSteerMotor().getVelocity().getValueAsDouble());
        });
    }

    public void periodic() {
        Logger.recordOutput(SWERVE.LOG_PATH+"Odometryvalid", swerve.odometryIsValid());
        // swerve.periodic();
    }

    public void simulationPeriodic() {
        Robot.FIELD.setRobotPose(getCurrentPosition());
    }

    /**
     * Stop the swerve (feed zeros for all target velocities)
     */
    protected void stop(){
        drive(new ChassisSpeeds());
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, robot-relative
     *
     * @param speeds the speeds to run the drivetrain at
     */
    protected void drive(ChassisSpeeds speeds){
        swerve.drive(speeds, false);
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, robot-relative
     *
     * @param speeds the speeds to run the drivetrain at
     */
    protected void drive(ChassisSpeeds speeds, DriveModes mode){
        swerve.drive(
            speeds,
            switch(mode){
                case FIELD_RELATIVE:
                    yield true;
                case AUTOMATIC:
                    yield !robotRelative;
                case ROBOT_RELATIVE:
                    yield false;
            }
        );
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
     * Set whether human-input-processed joystick input should be robot-relative
     * (as opposed to field-relative)
     *
     * @param robotRelative whether to run the drivetrain robot-relative
     */
    protected void setRobotRelative(boolean robotRelative){
        this.robotRelative = robotRelative;
    }

    /**
     * Define whatever direction the robot is facing as forward
     */
    protected void resetHeading(){
        swerve.resetHeading();
    }

    /**
     * Get the current robot yaw as a Rotation2d
     */
    public Rotation2d getYaw() {
        return swerve.getYaw();
    }

    /**
     * Get the current position of the swerve as judged by odometry.
     */
    public Pose2d getCurrentPosition() {
        return swerve.getCurrentOdometryPosition();
    }

    /**
     * Set the robot's known rotation.
     *
     * @param yaw the rotation to set as a Rotation2d
     */
    public void setGyroscopeRotation(Rotation2d yaw){
        swerve.setGyroscopeYaw(yaw);
    }

    /**
     * Reset the robot's known position.
     *
     * @param pose the pose to set the robot's known position to.
     */
    protected void resetPose(Pose2d pose) {
        swerve.setKnownOdometryPose(pose);
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
     * Use PID to snap the robot to a rotational setpoint
     *
     * @param setpoint the setpoint to snap to
     * @return the rotational velocity setpoint as a Rotation2d
     */
    protected double snapToAngle(Rotation2d setpoint) {
        double currYaw = Math.toRadians(getYaw().getDegrees()%360);
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
     * @param fieldRelativeAllowed if this is true, switch between field- and
     * robot-relative based on {@link Swerve#robotRelative}. Otherwise, force
     * robot-relative.
     *
     * @return a ChassisSpeeds ready to be sent to the swerve.
     */
    protected ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot){
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

        ChassisSpeeds currentSpeeds;

        if (isSlowMode) {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveRotate * SWERVE.ROTATE_POWER_SLOW * SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
            ));
        }
        else {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveRotate * SWERVE.ROTATE_POWER_FAST * SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
            ));
        }

        return currentSpeeds;
    }
}
