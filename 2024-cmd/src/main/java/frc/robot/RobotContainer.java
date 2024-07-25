// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.proxies.*;
import frc.robot.helpers.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import java.util.Set;

import com.NewtonSwerve.Gyro.NewtonPigeon2;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // The robot's subsystems
    private final Swerve swerve = new Swerve(new NewtonPigeon2(new Pigeon2(CAN.PIGEON_CAN_ID)));
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final LEDs leds = new LEDs();

    //Any helpers that need to be instantiated go here:
    private final PoseVision poseVision = new PoseVision(
        APRILTAG_VISION.kP,
        APRILTAG_VISION.kI,
        APRILTAG_VISION.kD,
        0
    );
    private final LimelightTargeting noteLock = new LimelightTargeting(
        NOTELOCK.LIMELIGHT_NAME, NOTELOCK.LOCK_ERROR,0,0,0,0
    );
    private final PIDController drivePID = new PIDController(
        NOTELOCK.DRIVE_TO_DRIVE_kP,
        NOTELOCK.DRIVE_TO_DRIVE_kI,
        NOTELOCK.DRIVE_TO_DRIVE_kD
    );
    private final PIDController turnPID = new PIDController(
        NOTELOCK.DRIVE_TO_TURN_kP,
        NOTELOCK.DRIVE_TO_TURN_kI,
        NOTELOCK.DRIVE_TO_TURN_kD
    );

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );
    private final CommandXboxController operatorController = new CommandXboxController(
        CONTROLLERS.OPERATOR_PORT
    );

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        configureDefaults();
        configureBindings();

        AutoManager.addSubsystems(swerve, intake, elevator, shooter, leds);
        AutoManager.loadAutos();
        AutoManager.broadcastChooser();
    }

    //Please use this.setDefaultCommand() instead of SubsystemBase.setDefaultCommand() here.
    //The method from this class includes a check for interruption behavior.

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        setDefaultCommand(swerve, swerve.commands.driveCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            () -> driverController.getRightX()
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Set the LED strip's default command to showing whether or not the robot is loaded
        setDefaultCommand(leds, leds.commands.indicateLoadedCommand(
            () -> shooter.isMiddleBeamBreakTripped()
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Snap the swerve to the set angle. Does NOT need to be left-right corrected
     *
     * @param angle Rotation2d containing the angle setpoint
     * @return the swerve.snapToCommand
     *
     * @apiNote this command never ends on its own; it must be interrupted to end
     */
    private Command snapToCommand(Rotation2d angle){
        return swerve.commands.snapToCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            Rotation2d.fromDegrees((360-angle.getDegrees())%360)
        );
    }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        // Slow Mode (hold)
        driverController.rightBumper()
        .onTrue(
            swerve.commands.slowModeCommand(true) // Enable slow mode
        ).onFalse(
            swerve.commands.slowModeCommand(false) // Disable slow mode
        );

        // Reset Gyroscope (press)
        driverController.back().onTrue(
            swerve.commands.zeroGyroscopeCommand()
        );

        //Autocollect (hold)
        driverController.a().whileTrue(swerve.commands.rawRotationCommand(
            () -> driverController.getLeftX(), // Drive forward and back freely
            () -> 0, // No side-to-side

            // Rotation speed of the autocollect function (turn towards the note)
            () -> noteLock.driveToTarget(
                turnPID,
                drivePID,
                NOTELOCK.TELEOP_DRIVE_TO_TARGET_ANGLE
            ).omegaRadiansPerSecond
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // Robot-oriented (hold)
        driverController.leftBumper().onTrue(
            swerve.commands.robotOrientedCommand(true) // Enable robot-oriented driving
        ).onFalse(
            swerve.commands.robotOrientedCommand(false) // Disable robot-oriented driving
        );

        // Amp-score or shoot (press) + force-shoot (hold)
        driverController.rightTrigger(0.1).whileTrue( // <-- The 0.1 is the threshold

            /*
             * This is a bit of a weird trigger. We want it to handle amp scoring, static shots, and vision shots.
             * To do that, we use a whileTrue trigger and override it where necessary. On the amp score, we always
             * override it with the ScheduleCommand. On the speaker score, we apply the whileTrue to a
             * WaitForCondition command, but then don't apply it to the the ShootCommand inside. This means that
             * it won't keep trying to shoot after the trigger is released, but it will finish shooting if the
             * trigger is released in the middle of the shot.
            */

            new DeferredCommand(
                () -> { // <-- Notice that we're starting a lambda here. This will cast to a Supplier<Command>.
                    if(elevator.isAmp()){

                        /*
                         * The ScheduleCommand frees the AmpScoreCommand from the whileTrue(),
                         * making the AmpScoreCommand able to run for as long as it wants without
                         * being cancelled by the user releasing the trigger. 
                         */
                        return new ScheduleCommand(
                            new AmpScoreCommand(shooter, elevator, intake, leds)
                        );
                    }
                    else{ // The elevator isn't in amp position, meaning we need to shoot
                        if(driverController.getHID().getXButton()){ // If we're force-shooting

                            // Ignore whatever else might have been happening and just shoot
                            return new OverrideEverythingCommand(
                                new ShootCommand(shooter, elevator, intake, leds)
                            );
                        }
                        else{

                            /*
                             * This WaitForConditionCommand will be cancelled if the user releases
                             * the trigger. However, it schedules the passed-in command instead of
                             * wrapping it, meaning the ShootCommand inside it will NOT be cancelled
                             * if the WaitForConditionCommand has already scheduled it.
                             */
                            return new WaitForConditionCommand(
                                () -> shooter.readyToShoot() && elevator.isAtTargetPosition(),
                                new ShootCommand(shooter, elevator, intake, leds)
                                .andThen(new StowCommand(shooter, elevator, intake))
                            );
                        }
                    }
                },
                Set.of(shooter, elevator, intake, leds) // Requirements for the DeferredCommand above
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Party Mode (hold)
        driverController.start().whileTrue(
            leds.commands.partyCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Pass-aim (hold)
        driverController.y().whileTrue(
            new DeferredCommand(
                () -> {
                    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
                        return snapToCommand(Rotation2d.fromDegrees(330)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                    }
                    else{
                        return snapToCommand(Rotation2d.fromDegrees(30)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                    }
                },
                Set.of(swerve)
            )
        );

        // Stow (press)
        driverController.b().onTrue(
            // This clears all scheduled commands and stows, meaning the robot
            // will stow without reference to what it was previously doing.
            new OverrideEverythingCommand(
                new StowCommand(shooter, elevator, intake)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf) //Cancel self so we don't have to wait for a full stow before moving on
            )
        );

        // Snap-to (hold)
        driverController.pov(0).whileTrue(
            snapToCommand(Rotation2d.fromDegrees(0))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        driverController.pov(90).whileTrue(
            snapToCommand(Rotation2d.fromDegrees(90))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        driverController.pov(180).whileTrue(
            snapToCommand(Rotation2d.fromDegrees(180))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        driverController.pov(270).whileTrue(
            snapToCommand(Rotation2d.fromDegrees(270))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );



        //Passthrough (hold)
        operatorController.rightTrigger(0.1).whileTrue(
            new PassThroughCommand(shooter, elevator, intake, leds)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        //Vision Prime (press)
        operatorController.rightBumper().onTrue(
            new PrimeCommand(
                // Lambda that returns the table entry corresponding to distance to the tag
                () -> RangeTable.get(poseVision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)),
                shooter, elevator, intake, leds,
                // For the honing lights
                () -> poseVision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        //Podium Prime (press)
        operatorController.b().onTrue(
            new PrimeCommand(
                RangeTable.getPodium(), shooter, elevator, intake, leds,
                // For the honing lights
                () -> poseVision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        // Outake (hold)
        operatorController.leftBumper().whileTrue(
            new OutakeCommand(shooter, intake)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Intake (press)
        operatorController.leftTrigger(0.1).onTrue(
            new IntakeCommand(shooter, elevator, intake, leds)
            .andThen(
                shooter.commands.primeCommand(RangeTable.getSubwoofer())
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Stow (press)
        operatorController.a().onTrue(
            new OverrideEverythingCommand(
                new StowCommand(shooter, elevator, intake)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf) //Cancel self so we don't have to wait for a full stow before moving on
            )
        );

        // Amp prime (press)
        operatorController.x().onTrue(
            elevator.commands.setStaticPositionCommand(
                ELEVATOR.PIVOT_ANGLE_AMP, ELEVATOR.EXTENSION_METERS_AMP
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Climb position (press)
        operatorController.y().onTrue(
            new ClimbCommand(elevator, intake, shooter)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Extend (hold)
        operatorController.pov(0).whileTrue(
            elevator.commands.incrementElevatorPositionCommand(0, ELEVATOR.MANUAL_EXTENSION_SPEED)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Retract (hold)
        operatorController.pov(180).whileTrue(
            elevator.commands.incrementElevatorPositionCommand(0, -ELEVATOR.MANUAL_EXTENSION_SPEED)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Note Request (hold)
        operatorController.back().whileTrue(
            leds.commands.blinkCommand(LEDS.YELLOW, 2)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Trap Prime (press)
        operatorController.start().onTrue(
            new PrimeCommand(RangeTable.getTrap(), shooter, elevator, intake, leds, () -> 0)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getAutonomousCommand();
    }

    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running).
     * <p> NOTE: all subsystems also have a setDefaultCommand method; this version includes a check for
     * default commands that cancel incoming commands that require the subsystem. Unless you're sure
     * of what you're doing, you should use this one.
     *
     * @param subsystem the subsystem to apply the default command to
     * @param command to command to set as default
     */
    private void setDefaultCommand(SubsystemBase subsystem, Command command){
        if(command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf){
            subsystem.setDefaultCommand(command);
        }
        else{
            //If you want to force-allow setting a cancel-incoming default command, directly call `subsystem.setDefaultCommand()` instead
            throw new UnsupportedOperationException("Can't set a default command that cancels incoming!");
        }
    }
}
