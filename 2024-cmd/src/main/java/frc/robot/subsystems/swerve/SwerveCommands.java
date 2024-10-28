package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.subsystems.SubsystemCommands;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

public class SwerveCommands extends SubsystemCommands{
    private Swerve swerve;
    public SwerveCommands(Swerve swerve){
        this.swerve = swerve;
    }

    /**
     * Command to drive the swerve based on the inputted lambdas until interrupted. Processes the inputs to
     * optimize for human comfort and control, so joystick inputs will almost always be the only thing
     * that should be passed in.
     *
     * @param suppliedX a lambda for forward-back translation input (will be optimized for human control)
     * @param suppliedY a lambda for left-right translation input (will be optimized for human control)
     * @param suppliedRot a lambda for rotation input (will be optimized for human control)
     * @param driveMode the drive mode to use (robot-relative, field-relative, or based on driver request)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command driveCommand(DoubleSupplier suppliedX, DoubleSupplier suppliedY, DoubleSupplier suppliedRot, DriveModes driveMode) {
        return swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                suppliedX.getAsDouble(),
                suppliedY.getAsDouble(),
                suppliedRot.getAsDouble()
            ), driveMode);
        });
    }

    /**
     * Command to drive the swerve based on the inputted lambdas until interrupted. Does not apply any processing
     * to the inputs; don't use this for human input.
     *
     * @param suppliedX a lambda for forward-back translation input
     * @param suppliedY a lambda for left-right translation input
     * @param suppliedRot a lambda for rotation input
     * @param driveMode the drive mode to use (robot-relative, field-relative, or based on driver request)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     * @apiNote Note that this command has X and Y inverted compared to the commands with human input optimization.
     */
    public Command rawDriveCommand(DoubleSupplier suppliedX, DoubleSupplier suppliedY, DoubleSupplier suppliedRot, DriveModes driveMode){
        return swerve.run(() -> {
            swerve.drive(
                new ChassisSpeeds(
                    suppliedX.getAsDouble(),
                    suppliedY.getAsDouble(),
                    suppliedRot.getAsDouble()
                ),
                driveMode
            );
        });
    }

    /**
     * Same as {@link SwerveCommands#driveCommand(DoubleSupplier, DoubleSupplier, DoubleSupplier)}, but doesn't
     * process the rotation for human input. Translation should come from joysticks while rotation usually
     * should be from a PID controller or other software control.
     *
     * @param translationXSupplier a lambda for left-right translation input (will be optimized for human control)
     * @param translationYSupplier a lambda for forward-back translation input (will be optimized for human control)
     * @param rotationSpeedSupplier a lambda for rotation input (will NOT be optimized for human control)
     * @param driveMode the drive mode to use (robot-relative, field-relative, or based on driver request)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command rawRotationCommand(
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSpeedSupplier,
        DriveModes driveMode){

        return swerve.run(() -> {
            // Process translation for human input
            ChassisSpeeds processed = swerve.processJoystickInputs(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                0
            );

            // Override whatever the human input processing spat out for rotation
            // with the output of the passed-in lambda.
            processed.omegaRadiansPerSecond = rotationSpeedSupplier.getAsDouble();
            swerve.drive(processed, driveMode);
        });
    }

    /**
     * Command to snap to the specified angle while driving with human input. The translation inputs
     * are processed for human comfort and control, so these should almost always be joystick inputs.
     *
     * @param translationXSupplier a lambda for left-right translation input (will be optimized for human
     * control)
     * @param translationYSupplier a lambda for forward-back translation input (will be optimized for human
     * control)
     * @param angle the angle to snap to 
     * @param driveMode the drive mode to use (robot-relative, field-relative, or based on driver request)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command snapToCommand(
        DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
        Rotation2d angle, DriveModes driveMode
    ){
        return this.rawRotationCommand(
            translationXSupplier, translationYSupplier, () -> swerve.snapToAngle(angle), driveMode
        );
    }

        /**
     * Command to snap to an updating angle while driving with human input. The translation inputs
     * are processed for human comfort and control, so these should almost always be joystick inputs.
     *
     * @param translationXSupplier a lambda for left-right translation input (will be optimized for human
     * control)
     * @param translationYSupplier a lambda for forward-back translation input (will be optimized for human
     * control)
     * @param angleSupplier a lambda that returns the angle to snap to
     *
     * @param driveMode the drive mode to use (robot-relative, field-relative, or based on driver request)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command snapToCommand(
        DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
        Supplier<Rotation2d> angleSupplier, DriveModes driveMode
    ){
        return this.rawRotationCommand(
            translationXSupplier,
            translationYSupplier,
            () -> swerve.snapToAngle(swerve.getYaw().plus(angleSupplier.get())),
            driveMode
        ).alongWith(
            Commands.run(() -> Logger.recordOutput(SHOOTER.LOG_PATH+"OffsetSupplierValue", angleSupplier.get().getDegrees()))
        );
    }

    /**
     * Command to stop the drivetrain.
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command stopCommand(){
        return swerve.runOnce(() -> {swerve.drive(new ChassisSpeeds());});
    }

    /**
     * Command to enable or disable slow mode for all inputs that are processed for
     * human comfort
     *
     * @param slowMode whether to enable (true) or disable (false) slow mode
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command slowModeCommand(boolean slowMode){
        // * Notice the Commands.runOnce() here instead of Swerve.runOnce(). This is so the slowModeCommand
        // * doesn't "require" the swerve subsystem, which means it can run while other commands (usually a
        // * driving command) run.
        return Commands.runOnce(() -> {
            swerve.setSlowMode(slowMode);
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", slowMode?"Slow mode enabled":"Slow mode disabled");
        }).ignoringDisable(true); // <-- this ensures the robot doesn't get stuck in SM if disabled while the button is held
    }

    /**
     * Command to enable or disable robot-relative control for all inputs that are processed for
     * human comfort
     *
     * @param robotRelative whether to enable (true) or disable (false) robot-relative control
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command robotRelativeCommand(boolean robotRelative){
        // See slowModeCommand above for comment on the Commands.runOnce
        return Commands.runOnce(() -> {
            swerve.setRobotRelative(robotRelative);
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", robotRelative?"Robot-relative enabled":"Robot-relative disabled");
        }).ignoringDisable(true);
    }

    /**
     * Command to reset the gyroscope yaw heading to zero
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command resetHeadingCommand() {
        // See slowModeCommand above for comment on the Commands.runOnce
        return Commands.runOnce(() -> {
            swerve.resetHeading();
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", "Swerve heading reset");
        });
    }

    /**
     * Command to set the odometry's known pose
     *
     * @param pose the pose to reset the odometry to
     *
     * @return The command
     *
     * @apiNote This command runs for one frame and
     * ends immediately
     */
    public Command setOdometryPoseCommand(Pose2d pose){
        return swerve.runOnce(() -> {
            swerve.resetPose(pose);
            Logger.recordOutput(SWERVE.LOG_PATH+"Console", (
                "Odometry pose reset to X: "
                +pose.getX()
                +", Y: "
                +pose.getY()
                +", and rotation (degrees): "
                +pose.getRotation().getDegrees()
            ));
            if(!Robot.isReal()){Robot.FIELD.setRobotPose(pose);}
        });
    }

    /**
     * Command to set the odometry's known pose with a pose from a blue
     * side path and dymanic mirroring to red.
     *
     * @param pose the pose to set (should be on the blue side)
     * @param flip lambda that returns whether to flip the path to red
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command setOdometryPoseCommand(Pose2d pose, BooleanSupplier flip){
        if(flip.getAsBoolean()){
            return setOdometryPoseCommand(
                new Pose2d(
                    MEASUREMENTS.FIELD_LENGTH_METERS - pose.getX(),
                    pose.getY(),
                    Rotation2d.fromRadians(Math.PI).minus(pose.getRotation())
                )
            );
        }
        else{
            return setOdometryPoseCommand(pose);
        }
    }

    /**
     * Command to run all the necessary setup for autonomous.
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command autonomousInitCommand(){
        return this.stopCommand().alongWith(this.resetHeadingCommand());
    }

    /**
     * Command to follow the given path.
     *
     * @param trajectory the path to follow
     * @param flip lambda that returns whether the path should be flipped to the red side of the field.
     *
     * @return the command
     *
     * @apiNote this command ends when the entire path has been followed. See FollowPathCommand in
     * {@link SwerveCommands}
     */
    public Command followPathCommand(Trajectory trajectory, BooleanSupplier flip){
        return new FollowPathCommand(trajectory, flip);
    }

    /**
     * Command to follow a path with the option to deviate from it as configured by the parameters
     *
     * @param trajectory the path to follow
     *
     * @param flip lambda that returns whether the path should be mirrored to the red side of the
     * field.
     *
     * @param useAlternateRotation a lambda that returns whether to use the alternate rotation
     * provided by {@code rotationSupplier}
     *
     * @param rotationSupplier a lambda that returns the alternate rotation to be used when
     * {@code useAlternateRotation} returns {@code true}. NOTE: The degree and radian values stored
     * in the Rotation2d are used as a velocity setpoint, not a position setpoint, so you have to run
     * your own control loop (as needed) for this to work.
     *
     * @param useAlternateTranslation a lambda that returns whether to use the alternate translation
     * provided by {@code translationSupplier}
     *
     * @param translationSupplier a lambda that returns the alternate translation to be used when
     * {@code useAlternatTranslation} returns {@code true}. The rotation component of the
     * {@code ChassisSpeeds} is ignored.
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


    private class FollowPathCommand extends Command{
        // Pathing variables
        private Trajectory trajectory;
        private Timer timer = new Timer();

        // Alternate movement variables
        private BooleanSupplier useAlternateRotation = () -> false;
        private Supplier<Rotation2d> alternateRotation = () -> new Rotation2d();
        private BooleanSupplier useAlternateTranslation = () -> false;
        private Supplier<ChassisSpeeds> alternateTranslation = () -> new ChassisSpeeds();

        // PID controllers
        private ProfiledPIDController turnController;
        private HolonomicDriveController drivePID;
        private PIDController xController;
        private PIDController yController;

        // Whether to flip to the red side of the field
        private BooleanSupplier flip;

        /**
         * Command to follow a trajectory
         *
         * @param trajectory the trajectory to follow
         * @param flip lambda that returns whether to mirror the path to the
         * red side of the field.
         */
        public FollowPathCommand(Trajectory trajectory, BooleanSupplier flip){
            this.trajectory = trajectory;

            this.xController = new PIDController(
                SWERVE.PATH_FOLLOW_TRANSLATE_kP,
                SWERVE.PATH_FOLLOW_TRANSLATE_kI,
                SWERVE.PATH_FOLLOW_TRANSLATE_kD
            );
            this.xController.setTolerance(
                SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
                SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
            );

            this.yController = new PIDController(
                SWERVE.PATH_FOLLOW_TRANSLATE_kP,
                SWERVE.PATH_FOLLOW_TRANSLATE_kI,
                SWERVE.PATH_FOLLOW_TRANSLATE_kD
            );
            this.yController.setTolerance(
                SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
                SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
            );

            this.turnController = new ProfiledPIDController(
                SWERVE.PATH_FOLLOW_ROTATE_kP,
                SWERVE.PATH_FOLLOW_ROTATE_kI,
                SWERVE.PATH_FOLLOW_ROTATE_kD,
                new Constraints(
                    SWERVE.PATH_FOLLOW_ROTATE_MAX_VELOCITY,
                    SWERVE.PATH_FOLLOW_ROTATE_MAX_ACCELLERATION
                )
            );
            this.turnController.setTolerance(
                SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
                SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
            );
            this.turnController.enableContinuousInput(-Math.PI, Math.PI);

            this.drivePID = new HolonomicDriveController(xController, yController, turnController);
            this.drivePID.setTolerance(
                new Pose2d(
                    new Translation2d(xController.getPositionTolerance(), yController.getPositionTolerance()),
                    new Rotation2d(turnController.getPositionTolerance())
                )
            );

            this.flip = flip;
        }

        /**
         * Command to follow a path with the option to deviate from it as configured by the parameters
         *
         * @param trajectory the path to follow
         *
         * @param flip lambda that returns whether the path should be mirrored to the red side of the
         * field.
         *
         * @param useAlternateRotation a lambda that returns whether to use the alternate rotation
         * provided by {@code rotationSupplier}
         *
         * @param rotationSupplier a lambda that returns the alternate rotation to be used when
         * {@code useAlternateRotation} returns {@code true}. NOTE: The degree and radian values stored
         * in the Rotation2d are used as a velocity setpoint, not a position setpoint, so you have to run
         * your own control loop (as needed) for this to work.
         *
         * @param useAlternateTranslation a lambda that returns whether to use the alternate translation
         * provided by {@code translationSupplier}
         *
         * @param translationSupplier a lambda that returns the alternate translation to be used when
         * {@code useAlternatTranslation} returns {@code true}. The rotation component of the
         * {@code ChassisSpeeds} is ignored.
         */
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

            // Stop the swerve
            swerve.drive(new ChassisSpeeds());
        }
        public void execute(){
            // Instances of State contain information about pose, velocity, accelleration, curvature, etc.
            State desiredState = trajectory.sample(timer.get());

            if(flip.getAsBoolean()){
                desiredState = flip(desiredState);
            }

            if(Robot.isReal()){
                Logger.recordOutput(SWERVE.LOG_PATH+"TargetPose", desiredState.poseMeters);

                ChassisSpeeds driveSpeeds = drivePID.calculate(
                    swerve.getCurrentPosition(),
                    desiredState,
                    desiredState.poseMeters.getRotation()
                );

                // Override the rotation speed (NOT position target) if useAlternativeRotation
                // returns true.
                if(useAlternateRotation.getAsBoolean()){
                    driveSpeeds.omegaRadiansPerSecond = alternateRotation.get().getRadians();
                }

                // Same, except overriding translation only
                if(useAlternateTranslation.getAsBoolean()){
                    driveSpeeds.vxMetersPerSecond = alternateTranslation.get().vxMetersPerSecond;
                    driveSpeeds.vyMetersPerSecond = alternateTranslation.get().vyMetersPerSecond;
                }

                swerve.drive(driveSpeeds);
            }
            else{
                // The swerve automatically sets the visible position on the simulated
                // field, so all we have to do is set its known position
                swerve.resetPose(desiredState.poseMeters);
            }
        }
        public void end(boolean interrupted){
            swerve.drive(new ChassisSpeeds());
        }
        public boolean isFinished(){
            return ( // Only return true if enough time has elapsed, we're at the target location, and we're not using alternate movement.
                timer.hasElapsed(trajectory.getTotalTimeSeconds())
                && (drivePID.atReference() || !Robot.isReal())
                && !useAlternateRotation.getAsBoolean()
                && !useAlternateTranslation.getAsBoolean()
            );
        }

        /**
         * Mirror a State object to the other side of the field.
         *
         * @param state {@code State}: the state to mirror
         * @return the mirrored state
         */
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
