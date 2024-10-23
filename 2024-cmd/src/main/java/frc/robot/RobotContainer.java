// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.Controls.ControlSets;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.proxies.*;
import frc.robot.helpers.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Positions;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class RobotContainer {
    // The robot's subsystems
    private final Swerve swerve;
    private final Shooter shooter;
    private final Intake intake;
    private final Elevator elevator;
    private final LEDs leds;

    // Helpers
    private final PoseVision poseVision;
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

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        swerve = Swerve.instantiate();
        intake = Intake.instantiate();
        elevator = Elevator.instantiate();
        shooter = Shooter.instantiate();
        leds = LEDs.instantiate();
        poseVision = PoseVision.instantiate(
            APRILTAG_VISION.kP,
            APRILTAG_VISION.kI,
            APRILTAG_VISION.kD,
            0
        );

        configureBindings(ControlSets.MAIN_TELEOP);
        configureDefaults();


        AutoManager.prepare();
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        setDefaultCommand(swerve, swerve.commands.driveCommand(
            Controls.driveTranslateX, Controls.driveTranslateY, Controls.driveRotate, DriveModes.AUTOMATIC
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Set the LED strip's default command to showing whether or not the robot is loaded
        setDefaultCommand(leds, leds.commands.indicateLoadedCommand(Suppliers.robotHasNote)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Configure all button bindings
     *
     * @param controlSet the set of controls to use
     */
    private void configureBindings(ControlSets controlSet) {
        CommandScheduler.getInstance().getDefaultButtonLoop().clear();
        Controls.applyControlSet(controlSet);

        Controls.slowMode.onTrue(
            swerve.commands.slowModeCommand(true) // Enable slow mode
        ).onFalse(
            swerve.commands.slowModeCommand(false) // Disable slow mode
        );

        Controls.zeroGryoscope.onTrue(
            swerve.commands.resetHeadingCommand()
        );

        Controls.autocollect.whileTrue(
            swerve.commands.rawRotationCommand(
                () -> 0, // No side-to-side
                Controls.driveTranslateY, // Drive forward and back freely

                // Rotation speed of the autocollect function (turn towards the note)
                () -> noteLock.driveToTarget(
                    turnPID,
                    drivePID,
                    NOTELOCK.TELEOP_DRIVE_TO_TARGET_ANGLE
                ).omegaRadiansPerSecond,
                DriveModes.ROBOT_RELATIVE
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.robotRelative.onTrue(
            swerve.commands.robotRelativeCommand(true) // Enable robot-oriented driving
        ).onFalse(
            swerve.commands.robotRelativeCommand(false) // Disable robot-oriented driving
        );

        Controls.score.onTrue(
            new ShootCommand(
                RangeTable.getSubwoofer(),
                () -> Controls.score.getAsBoolean(),
                Suppliers.offsetFromSpeakerTag
            ).onlyIf(() -> elevator.isTargeting(Positions.STOWED))
        );

        Controls.partyMode.toggleOnTrue(
            leds.commands.partyCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.passAim.whileTrue(
            // This command is deferred to make the call to Suppliers.robotRunningOnRed
            // happen when we try to pass-aim, rather than when the robot boots
            new DeferredCommand(
                () -> swerve.commands.snapToCommand(
                    Controls.driveTranslateX,
                    Controls.driveTranslateY,
                    Rotation2d.fromDegrees(
                        Suppliers.robotRunningOnRed.getAsBoolean() ? 330 : 30
                    ),
                    DriveModes.AUTOMATIC
                ),
                Set.of(swerve)
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.stow.onTrue(
            // This clears all scheduled commands and stows, meaning the robot
            // will stow without reference to what it was previously doing.
            new OverrideEverythingCommand(
                new StowCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf) // Cancel self so we don't have to wait for a full stow before moving on
            )
        );

        Controls.autoAim.whileTrue(
            swerve.commands.rawRotationCommand(
                Controls.driveTranslateX, Controls.driveTranslateY, Suppliers.aimToSpeakerPidLoopPositiveSearch, DriveModes.AUTOMATIC
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.snapForward.whileTrue(
            swerve.commands.snapToCommand(Controls.driveTranslateX, Controls.driveTranslateY, Rotation2d.fromDegrees(0), DriveModes.AUTOMATIC)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapBack.whileTrue(
            swerve.commands.snapToCommand(Controls.driveTranslateX, Controls.driveTranslateY, Rotation2d.fromDegrees(180), DriveModes.AUTOMATIC)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapLeft.whileTrue(
            swerve.commands.snapToCommand(Controls.driveTranslateX, Controls.driveTranslateY, Rotation2d.fromDegrees(270), DriveModes.AUTOMATIC)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapRight.whileTrue(
            swerve.commands.snapToCommand(Controls.driveTranslateX, Controls.driveTranslateY, Rotation2d.fromDegrees(90), DriveModes.AUTOMATIC)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );



        Controls.passThrough.whileTrue(
            new PassThroughCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.visionShoot.onTrue(
            new ShootCommand(
                Suppliers.bestRangeEntry,
                () -> Controls.score.getAsBoolean(),
                Suppliers.offsetFromSpeakerTag
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.podiumShoot.onTrue(
            new ShootCommand(
                RangeTable.getPodium(),
                () -> Controls.score.getAsBoolean(),
                Suppliers.offsetFromSpeakerTag
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.outake.whileTrue(
            new OutakeCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.intake.onTrue(
            new IntakeCommand().andThen(
                shooter.commands.primeCommand(RangeTable.getSubwoofer())
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        Controls.ampScore.onTrue(
            new AmpScoreCommand(() -> Controls.score.getAsBoolean())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.climb.onTrue(
            new ClimbCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        Controls.extendElevator.whileTrue(
            elevator.commands.incrementElevatorPositionCommand(0, ELEVATOR.MANUAL_EXTENSION_SPEED)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.retractElevator.whileTrue(
            elevator.commands.incrementElevatorPositionCommand(0, -ELEVATOR.MANUAL_EXTENSION_SPEED)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.noteRequest.whileTrue(
            leds.commands.blinkCommand(LEDS.YELLOW, 2)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.trapShoot.onTrue(
            new PrimeCommand(RangeTable.getTrap(), () -> 0)
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
