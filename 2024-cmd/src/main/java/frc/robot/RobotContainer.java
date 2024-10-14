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

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {
    // The robot's subsystems
    private final Swerve swerve;
    private final Shooter shooter;
    private final Intake intake;
    private final Elevator elevator;
    private final LEDs leds;

    //Helpers
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

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );
    private final CommandXboxController operatorController = new CommandXboxController(
        CONTROLLERS.OPERATOR_PORT
    );

    // Useful suppliers that are private to RobotContainer (can't go in Suppliers)
    private DoubleSupplier translateX = () -> -driverController.getLeftX();
    private DoubleSupplier translateY = () -> -driverController.getLeftY();
    private DoubleSupplier rotate = () -> -driverController.getRightX();

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

        configureDefaults();

        Controls.addControllers(driverController, operatorController);
        configureBindings(ControlSets.MAIN_TELEOP);

        AutoManager.prepare();
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        setDefaultCommand(swerve, swerve.commands.driveCommand(
            translateX, translateY, rotate, DriveModes.AUTOMATIC
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Set the LED strip's default command to showing whether or not the robot is loaded
        setDefaultCommand(leds, leds.commands.indicateLoadedCommand(Suppliers.robotHasNote)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
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
            translateX, translateY,
            Rotation2d.fromDegrees((360-angle.getDegrees())%360),
            DriveModes.AUTOMATIC
        );
    }

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
            swerve.commands.zeroGyroscopeCommand()
        );

        Controls.autocollect.whileTrue(
            swerve.commands.rawRotationCommand(
                () -> 0, // No side-to-side
                translateY, // Drive forward and back freely

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
            ).unless(() -> elevator.isTargeting(Positions.AMP))
        );

        Controls.partyMode.whileTrue(
            leds.commands.partyCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.passAim.whileTrue(
            new ConditionalCommand(
                snapToCommand(Rotation2d.fromDegrees(330)),
                snapToCommand(Rotation2d.fromDegrees(30)),
                Suppliers.robotRunningOnRed
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
                this.translateX, this.translateY, Suppliers.aimToSpeakerPidLoopPositiveSearch, DriveModes.AUTOMATIC
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.snapForward.whileTrue(
            snapToCommand(Rotation2d.fromDegrees(0))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapBack.whileTrue(
            snapToCommand(Rotation2d.fromDegrees(180))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapLeft.whileTrue(
            snapToCommand(Rotation2d.fromDegrees(270))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapRight.whileTrue(
            snapToCommand(Rotation2d.fromDegrees(90))
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
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        Controls.podiumShoot.onTrue(
            new ShootCommand(
                RangeTable.getPodium(),
                () -> Controls.score.getAsBoolean(),
                Suppliers.offsetFromSpeakerTag
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
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
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
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
