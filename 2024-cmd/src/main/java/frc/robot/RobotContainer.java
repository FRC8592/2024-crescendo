// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.proxies.*;
import frc.robot.helpers.*;
import frc.robot.subsystems.*;

import com.NewtonSwerve.Gyro.NewtonPigeon2;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kP,
        APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kI,
        APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kD
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

    //Run config functions here
    public RobotContainer() {
        configureDefaults();
        configureBindings();

        Autos.broadcastChooser();
    }

    //Please use this.setDefaultCommand() instead of SubsystemBase.setDefaultCommand() here.
    //The method from this class includes a check for interruption behavior.

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        setDefaultCommand(swerve, swerve.driveCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            () -> driverController.getRightX()
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Set the LED strip's default command to showing whether or not the robot is loaded
        setDefaultCommand(leds, leds.indicateLoadedCommand(
            () -> shooter.isMiddleBeamBreakTripped()
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Snap the swerve to the set angle. Does NOT need to be left-right corrected
     *
     * @param angle Rotation2d containing the angle setpoint
     * @return the swerve.snapToCommand
     */
    private Command snapToCommand(Rotation2d angle){
        return swerve.snapToCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            Rotation2d.fromDegrees((360-angle.getDegrees())%360)
        );
    }

    /**
     * Configure all button bindings here
     */
    private void configureBindings() {
        // Translate on Driver Left Stick -------------- DONE
        // Rotate on Driver Right Stick ---------------- DONE
        // Slow Mode on Driver Right Bumper ------------ DONE
        // Reset Gyro on Driver Back ------------------- DONE
        // Auto Collect on Driver A -------------------- DONE
        // Robot Oriented on Driver Left Bumper -------- DONE
        // Score on Driver Right Trigger --------------- DONE
        // Party Mode on Driver Start ------------------ DONE
        // Pass Aim on Driver Y ------------------------ DONE
        // Force Shoot on Driver X --------------------- DONE
        // Snap-to on Driver DPAD ---------------------- DONE

        // Passthrough on Operator Right Trigger ------- DONE
        // Range Table Shoot on Operator Right Bumper -- DONE
        // Shoot From Podium on Operator B ------------- DONE
        // Outake on Operator Left Bumper -------------- DONE
        // Intake on Operator Left Trigger ------------- DONE
        // Stow on Operator A or Driver B -------------- DONE
        // Amp on Operator X --------------------------- DONE
        // Climb on Operator Y ------------------------- DONE
        // Manual Extend on Operator DPAD Up ----------- DONE
        // Manual Retract on Operator DPAD Down -------- DONE
        // Led Amp Signal on Operator Back ------------- DONE
        // Trap Prime on Operator Start ---------------- DONE

        // Slow Mode (hold)
        driverController.rightBumper()
        .onTrue(
            swerve.slowModeCommand(true) // Enable slow mode
        ).onFalse(
            swerve.slowModeCommand(false) // Disable slow mode
        );

        // Reset Gyroscope (press)
        driverController.back().onTrue(
            swerve.zeroGyroscopeCommand()
        );

        //Autocollect (hold)
        driverController.a().whileTrue(swerve.noExpoRotationCommand(
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
            swerve.robotOrientedCommand(true) // Enable robot-oriented driving
        ).onFalse(
            swerve.robotOrientedCommand(false) // Disable robot-oriented driving
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

            elevator.isAmp()
            ?( // This runs if the elevator is in the amp position
                new ScheduleCommand(
                    new AmpScoreCommand(shooter, elevator, intake, leds)
                    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                )
            )
            :( // This block runs if the elevator is NOT in the amp position

                driverController.getHID().getXButton()
                ?( //If X button (force-shoot) pressed,

                    //Override any elevator positioning that might have been happening and shoot
                    new OverrideEverythingCommand(
                        new ShootCommand(shooter, elevator, intake, leds)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                    )
                )
                :( // If the force-shoot button is NOT pressed,

                    // Refuse to shoot if we're not reasonably prepared
                    new WaitForConditionCommand(
                        () -> shooter.readyToShoot() && elevator.isAtTargetPosition(),
                        new ShootCommand(shooter, elevator, intake, leds)
                        .andThen(new StowCommand(shooter, elevator, intake))
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                    )
                )
            )
        );

        // Party Mode (hold)
        driverController.start().whileTrue(
            leds.partyCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        try{
            // Pass-aim (hold)
            driverController.y().whileTrue(
                snapToCommand(
                    Rotation2d.fromDegrees(
                        DriverStation.getAlliance().get() == Alliance.Red ? 330 : 30
                    )
                ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            );
        }catch(Exception e){}

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
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        //Podium Prime (press)
        operatorController.b().onTrue(
            new PrimeCommand(
                RangeTable.getPodium(), shooter, elevator, intake, leds,
                // For the honing lights
                () -> poseVision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Outake (hold)
        operatorController.leftBumper().whileTrue(
            new OutakeCommand(shooter, intake)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Intake (press)
        operatorController.leftTrigger(0.1).onTrue(
            new IntakeCommand(shooter, elevator, intake, leds)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
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
            elevator.setStaticPositionCommand(
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
            elevator.incrementElevatorPositionCommand(0, ELEVATOR.MANUAL_EXTENSION_SPEED)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Retract (hold)
        operatorController.pov(180).whileTrue(
            elevator.incrementElevatorPositionCommand(0, -ELEVATOR.MANUAL_EXTENSION_SPEED)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Note Request (hold)
        operatorController.back().whileTrue(
            leds.blinkCommand(LEDS.YELLOW, 2)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // Trap Prime (press)
        operatorController.start().onTrue(
            new PrimeCommand(RangeTable.getTrap(), shooter, elevator, intake, leds, () -> 0)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        // TODO finish autoInit and figure out the auto framework
        return null;//swerve.autonomousInit().andThen(Autos.exampleAuto(swerve, shooter, elevator, intake, leds));
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
