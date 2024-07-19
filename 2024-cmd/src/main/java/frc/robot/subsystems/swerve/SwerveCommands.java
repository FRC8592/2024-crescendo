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
     * @param suppliedX a lambda for X translation input (will be optimized for human control)
     * @param suppliedY a lambda for Y translation input (will be optimized for human control)
     * @param suppliedRot a lambda for rotation input (will be optimized for human control)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command driveCommand(DoubleSupplier suppliedX, DoubleSupplier suppliedY, DoubleSupplier suppliedRot) {
        return swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                suppliedX.getAsDouble(),
                suppliedY.getAsDouble(),
                suppliedRot.getAsDouble(),
                true
            ));
        });
    }

    /**
     * Command to drive the swerve based on the inputted lambdas until interrupted. Does not apply any processing
     * to the inputs; don't use this for human input.
     *
     * @param suppliedX a lambda for X translation input
     * @param suppliedY a lambda for Y translation input
     * @param suppliedRot a lambda for rotation input
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command rawDriveCommand(DoubleSupplier suppliedX, DoubleSupplier suppliedY, DoubleSupplier suppliedRot) {
        return swerve.run(() -> {
            swerve.drive(
                new ChassisSpeeds(
                    suppliedX.getAsDouble(),
                    suppliedY.getAsDouble(),
                    suppliedRot.getAsDouble()
                )
            );
        });
    }

    /**
     * Command to snap to the specified angle while driving with human input. The translation inputs
     * are processed for human comfort and control, so these should almost always be joystick inputs.
     *
     * @param translationXSupplier a lambda for X translation input (will be optimized for human
     * control)
     * @param translationYSupplier a lambda for Y translation input (will be optimized for human
     * control)
     *
     * @param angle the angle to snap to (doesn't have any corrections applied; this should be left-right
     * inverted).
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command snapToCommand(
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        Rotation2d angle){
        return swerve.run(() -> {
            ChassisSpeeds processed = swerve.processJoystickInputs(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                0,
                true
            );

            processed.omegaRadiansPerSecond = swerve.snapToAngle(angle);
            swerve.drive(processed);
        });
    }

    /**
     * Same as {@link Swerve#driveCommand(DoubleSupplier, DoubleSupplier, DoubleSupplier)}, but doesn't
     * process the rotation for human input. Translation should come from joysticks while rotation usually
     * should be from a PID controller or other computer control.
     *
     * @param translationXSupplier a lambda for X translation input (will be optimized for human control)
     * @param translationYSupplier a lambda for Y translation input (will be optimized for human control)
     * @param rotationSpeedSupplier a lambda for rotation input (will NOT be optimized for human control)
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command rawRotationCommand(
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSpeedSupplier){

        return swerve.run(() -> {
            // Process translation for human input
            ChassisSpeeds processed = swerve.processJoystickInputs(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                0,
                false
            );

            // Override whatever the human input processing spat out for rotation
            // with the output of the passed-in lambda.
            processed.omegaRadiansPerSecond = rotationSpeedSupplier.getAsDouble();
            swerve.drive(processed);
        });
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
        });
    }

    /**
     * Command to enable or disable robot-oriented control for all inputs that are processed for
     * human comfort
     *
     * @param robotOriented whether to enable (true) or disable (false) robot-oriented control
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command robotOrientedCommand(boolean robotOriented){
        // See slowModeCommand above for comment on the Commands.runOnce
        return Commands.runOnce(() -> {
            swerve.setRobotOriented(robotOriented);
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
        // See slowModeCommand above for comment on the Commands.runOnce
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
        // We use Commands.runOnce here because swerve.stopCommand() requires the
        // swerve subsystem, meaning the alongWith() method would throw an exception
        // if it were swerve.runOnce() (because we'd be trying to run two commands that
        // both require the same subsystem at once).

        return Commands.runOnce(() -> {
            swerve.resetEncoders();
            swerve.resetPose(new Pose2d());
            swerve.resetToAbsEncoders();
            swerve.setThrottleCurrentLimit(POWER.SWERVE_AUTO_THROTTLE_CURRENT_LIMIT);
        }).alongWith(
            this.stopCommand()
        ).alongWith(
            this.zeroGyroscopeCommand()
        );
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
                ChassisSpeeds driveSpeeds = drivePID.calculate(
                    swerve.getCurrentPos(),
                    desiredState,
                    desiredState.poseMeters.getRotation()
                );

                // Override the rotation speed (NOT position target) if useAlternativeRotation
                // returns true.
                if(useAlternateRotation.getAsBoolean()){
                    driveSpeeds.omegaRadiansPerSecond = alternateRotation.get().getRadians();
                }

                if(useAlternateTranslation.getAsBoolean()){
                    driveSpeeds.vxMetersPerSecond = alternateTranslation.get().vxMetersPerSecond;
                    driveSpeeds.vyMetersPerSecond = alternateTranslation.get().vyMetersPerSecond;
                }

                swerve.drive(driveSpeeds);
            }
            else{
                //If simulated, set the robot's position on the simulated field
                Robot.FIELD.setRobotPose(desiredState.poseMeters);
            }
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
