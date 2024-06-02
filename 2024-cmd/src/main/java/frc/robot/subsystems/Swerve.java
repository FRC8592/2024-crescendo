// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import com.NewtonSwerve.*;
import com.NewtonSwerve.Mk4.*;
import com.NewtonSwerve.Gyro.Gyro;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import frc.robot.*;
import frc.robot.helpers.*;

public class Swerve extends NewtonSubsystem {
    private Mk4ModuleConfiguration swerveConfig;
    private NewtonSwerve swerve;
    private PIDController snapToController;

    private boolean isSlowMode;
    private boolean robotOriented;
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

    /**
     * Command to drive the swerve based on the inputted lambdas until interrupted. Processes the inputs to
     * optimize for human comfort and control, so joystick inputs will almost always be the only thing
     * that should be passed in.
     *
     * @param suppliedX {@code DoubleSupplier}: a lambda for X translation input (will be optimized for human control)
     * @param suppliedY {@code DoubleSupplier}: a lambda for Y translation input (will be optimized for human control)
     * @param suppliedRot {@code DoubleSupplier}: a lambda for rotation input (will be optimized for human control)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command driveCommand(DoubleSupplier suppliedX, DoubleSupplier suppliedY, DoubleSupplier suppliedRot) {
        return run(() -> {
            swerve.drive(processJoystickInputs(
                suppliedX.getAsDouble(),
                suppliedY.getAsDouble(),
                suppliedRot.getAsDouble(),
                true
            ));
        });
    }

    /**
     * Command to snap to the specified angle while driving with human input. The translation inputs
     * are processed for human comfort and control, so these should almost always be joystick inputs.
     *
     * @param translationXSupplier {@code DoubleSupplier}: a lambda for X translation input (will be
     * optimized for human control)
     * @param translationYSupplier {@code DoubleSupplier}: a lambda for Y translation input (will be
     * optimized for human control)
     *
     * @param angle {@code Rotation2d}: the angle to snap to (doesn't have any corrections applied)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command snapToCommand(
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        Rotation2d angle){
        return run(() -> {
            ChassisSpeeds processed = processJoystickInputs(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                0,
                true
            );

            processed.omegaRadiansPerSecond = snapToAngle(angle);
            swerve.drive(processed);
        });
    }

    /**
     * Same as {@link Swerve#driveCommand(DoubleSupplier, DoubleSupplier, DoubleSupplier)}, but doesn't
     * process the rotation for human input. Translation should come from joysticks while rotation usually
     * should be from a PID controller or other computer control.
     *
     * @param translationXSupplier {@code DoubleSupplier}: a lambda for X translation input (will be optimized for human control)
     * @param translationYSupplier {@code DoubleSupplier}: a lambda for Y translation input (will be optimized for human control)
     * @param rotationSpeedSupplier {@code DoubleSupplier}: a lambda for rotation input (will NOT be optimized for human control)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command noExpoRotationCommand(
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSpeedSupplier){

        return run(() -> {
            ChassisSpeeds processed = processJoystickInputs(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                0,
                false
            );

            processed.omegaRadiansPerSecond = rotationSpeedSupplier.getAsDouble();
            swerve.drive(processed);
        });
    }

    /**
     * Command to send the {@code ChassisSpeeds} object to the swerve. Mostly intended to be
     * used to stop the drivetrain
     *
     * @param speeds {@code ChassisSpeeds}: speeds to apply to the swerve
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command stopCommand(){
        return runOnce(() -> {swerve.drive(new ChassisSpeeds());});
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

    /**
     * Command to enable or disable slow mode for all inputs that are processed for
     * human comfort
     *
     * @param slowMode {@code boolean}: whether to enable (true) or disable (false) slow mode
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command slowModeCommand(boolean slowMode){
        return Commands.runOnce(() -> {
            this.isSlowMode = slowMode;
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", slowMode?"Slow mode enabled":"Slow mode disabled");
        });
    }

    /**
     * Command to enable or disable robot-oriented control for all inputs that are processed for
     * human comfort
     *
     * @param robotOriented {@code boolean}: whether to enable (true) or disable (false) robot-oriented control
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command robotOrientedCommand(boolean robotOriented){
        return Commands.runOnce(() -> {
            this.robotOriented = robotOriented;
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", robotOriented?"Robot-oriented enabled":"Robot-oriented disabled");
        });
    }

    /**
     * Command to reset the gyroscope yaw heading to zero
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command zeroGyroscopeCommand() {
        return Commands.runOnce(() -> {
            swerve.zeroGyroscope();
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Gyroscope zeroed");
        });
    }

    /**
     * Command to run all the necessary setup for autonomous. Please don't run this
     * any other time.
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command autonomousInitCommand(){
        return Commands.runOnce(() -> {
            this.resetEncoder();
            this.resetPose(new Pose2d());
            this.resetToAbsEncoders();
            this.setThrottleCurrentLimit(POWER.SWERVE_AUTO_THROTTLE_CURRENT_LIMIT);
        }).alongWith(
            this.stopCommand()
        ).alongWith(
            this.zeroGyroscopeCommand()
        );
    }

    /**
     * Command to follow the given path.
     *
     * @param trajectory {@code Trajectory}: the path to follow
     * @return the command
     *
     * @apiNote this command ends when the entire path has been followed. See the FollowPathCommand class in {@link Swerve}
     */
    public Command followPathCommand(Trajectory trajectory, BooleanSupplier flip){
        return new FollowPathCommand(trajectory, flip);
    }

    /**
     * Command to follow a path with the option to deviate from it as configured by the parameters
     *
     * @param trajectory {@code Trajectory}: the path to follow
     *
     * @param useAlternateRotation {@code BooleanSupplier}: a lambda that returns whether to use
     * the alternate rotation provided by {@code rotationSupplier}
     *
     * @param rotationSupplier {@code Supplier<Rotation2d>}: a lambda that returns the alternate
     * rotation to be used when {@code useAlternateRotation} returns {@code true}. NOTE: The degree
     * and radian values stored in the Rotation2d are used as a velocity setpoint, not a position
     * setpoint, so you have to run your own control loop (as needed) for this to work.
     *
     * @param useAlternateTranslation {@code BooleanSupplier}: a lambda that returns whether to use
     * the alternate translation provided by {@code translationSupplier}
     *
     * @param translationSupplier {@code Supplier<ChassisSpeeds}: a lambda that returns the alternate
     * translation to be used when {@code useAlternatTranslation} returns {@code true}. The rotation
     * component of the {@code ChassisSpeeds} is ignored.
     */
    public Command followPathCommand(
        Trajectory trajectory, BooleanSupplier flip,
        BooleanSupplier useAlternateRotation, Supplier<Rotation2d> rotationSupplier,
        BooleanSupplier useAlternateTranslation, Supplier<ChassisSpeeds> translationSupplier
    ){
        return new FollowPathCommand(
            trajectory, flip,
            useAlternateRotation, rotationSupplier,
            useAlternateTranslation, translationSupplier
        );
    }

    public void periodic() {
        Logger.recordOutput(SWERVE.LOG_PATH+"OdometryPosition", getCurrentPos());
    }

    public void simulationPeriodic() {
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

    private ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot, boolean fieldOrientedAllowed){
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
            currentSpeeds, (
                robotOriented || !fieldOrientedAllowed
                ? new Rotation2d()
                : swerve.getGyroscopeRotation()
            )
        );

        return currentSpeeds;
    }

    private class FollowPathCommand extends Command{
        private Trajectory trajectory;
        private Timer timer = new Timer();

        private BooleanSupplier useAlternateRotation = () -> {return true;};
        private Supplier<Rotation2d> alternateRotation = () -> {return new Rotation2d();};
        private BooleanSupplier useAlternateTranslation = () -> {return false;};
        private Supplier<ChassisSpeeds> alternateTranslation = () -> {return new ChassisSpeeds();};

        private ProfiledPIDController turnController;
        private HolonomicDriveController drivePID;
        private PIDController xController;
        private PIDController yController;

        private BooleanSupplier flip;

        public FollowPathCommand(Trajectory trajectory, BooleanSupplier flip){
            this.trajectory = trajectory;

            this.xController = new PIDController(
                SWERVE.PATH_FOLLOW_DRIVE_kP,
                SWERVE.PATH_FOLLOW_DRIVE_kI,
                SWERVE.PATH_FOLLOW_DRIVE_kD
            );
            this.xController.setTolerance(
                SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
                SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
            );

            this.yController = new PIDController(
                SWERVE.PATH_FOLLOW_DRIVE_kP,
                SWERVE.PATH_FOLLOW_DRIVE_kI,
                SWERVE.PATH_FOLLOW_DRIVE_kD
            );
            this.yController.setTolerance(
                SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
                SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
            );

            this.turnController = new ProfiledPIDController(
                SWERVE.PATH_FOLLOW_STEER_kP,
                SWERVE.PATH_FOLLOW_STEER_kI,
                SWERVE.PATH_FOLLOW_STEER_kD,
                new Constraints(
                    SWERVE.PATH_FOLLOW_STEER_MAX_VELOCITY,
                    SWERVE.PATH_FOLLOW_STEER_MAX_ACCELLERATION
                )
            );
            this.turnController.setTolerance(
                SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
                SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
            );
            this.turnController.enableContinuousInput(-Math.PI, Math.PI);

            this.drivePID = new HolonomicDriveController(xController, yController, turnController);

            this.flip = flip;
        }

        public FollowPathCommand(
            Trajectory trajectory, BooleanSupplier flip,
            BooleanSupplier useAlternateRotation, Supplier<Rotation2d> rotationSupplier,
            BooleanSupplier useAlternateTranslation, Supplier<ChassisSpeeds> translationSupplier
        ){
            this(trajectory, flip);
            this.useAlternateRotation = useAlternateRotation;
            this.alternateRotation = rotationSupplier;
            this.useAlternateRotation = useAlternateTranslation;
            this.alternateTranslation = translationSupplier;
        }

        public void initialize(){
            timer.reset();
            timer.start();
            Swerve.this.swerve.drive(new ChassisSpeeds());
        }
        public void execute(){
            State desiredState = trajectory.sample(timer.get());
            if(flip.getAsBoolean()){
                desiredState = flip(desiredState);
            }
            if(Robot.isReal()){
                ChassisSpeeds driveSpeeds = drivePID.calculate(
                    getCurrentPos(),
                    desiredState,
                    desiredState.poseMeters.getRotation()
                );

                if(useAlternateRotation.getAsBoolean()){
                    driveSpeeds.omegaRadiansPerSecond = alternateRotation.get().getRadians();
                }

                if(useAlternateTranslation.getAsBoolean()){
                    driveSpeeds.vxMetersPerSecond = alternateTranslation.get().vxMetersPerSecond;
                    driveSpeeds.vyMetersPerSecond = alternateTranslation.get().vyMetersPerSecond;
                }

                Swerve.this.swerve.drive(driveSpeeds);
            }
            else{
                Robot.FIELD.setRobotPose(desiredState.poseMeters);
            }
        }
        public boolean isFinished(){
            return ( // Only return true if enough time has elapsed, we're at the target location, and we're not using alternate movement.
                timer.hasElapsed(trajectory.getTotalTimeSeconds())
                // && (drivePID.atReference() || !Robot.isReal())
                // && !useAlternateRotation.getAsBoolean()
                // && !useAlternateTranslation.getAsBoolean()
            );
        }

        private State flip(State state){
            return new State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                new Pose2d(
                    MEASUREMENTS.FIELD_LENGTH_METERS - state.poseMeters.getX(),
                    state.poseMeters.getY(),
                    Rotation2d.fromRadians(Math.PI).minus(state.poseMeters.getRotation())
                ),
                -state.curvatureRadPerMeter
            );
        }
    }
}
