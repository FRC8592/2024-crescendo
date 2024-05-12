// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.helpers.*;
import frc.robot.subsystems.*;

import com.NewtonSwerve.Gyro.NewtonPigeon2;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Swerve swerve = new Swerve(new NewtonPigeon2(new Pigeon2(CAN.PIGEON_CAN_ID)));
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final LEDs leds = new LEDs();

    //NOT a subsystem
    private final PoseVision poseVision = new PoseVision(
        APRILTAG_VISION.kP,
        APRILTAG_VISION.kI,
        APRILTAG_VISION.kD,
        0
    );

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(
            CONTROLLERS.DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(
            CONTROLLERS.OPERATOR_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureDefaults();
        configureBindings();
    }

    private void configureDefaults(){
        setDefaultCommand(swerve, swerve.driveCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            () -> driverController.getRightX()
        ));

        setDefaultCommand(leds, leds.defaultCommand());
    }

    private Command snapToCommand(Rotation2d angle){
        return swerve.snapToCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            Rotation2d.fromDegrees((360-angle.getDegrees())%360)
        );
    }

    private void configureBindings() {
        // Translate on Driver Left Stick -------------- DONE
        // Rotate on Driver Right Stick ---------------- DONE
        // Slow Mode on Driver Right Bumper ------------ DONE
        // Reset Gyro on Driver Back ------------------- DONE
        // Auto Collect on Driver A -------------------- NOT DONE
        // Robot Oriented on Driver Left Bumper -------- DONE
        // Score on Driver Right Trigger --------------- DONE
        // Party Mode on Driver Start ------------------ DONE
        // Pass Aim on Driver Y ------------------------ DONE
        // Force Shoot on Driver X --------------------- NOT DONE
        // Snap-to on Driver DPAD ---------------------- DONE

        // Passthrough on Operator Right Trigger ------- NOT DONE
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

        driverController.rightBumper()
                .onTrue(swerve.slowModeCommand(true))
                .onFalse(swerve.slowModeCommand(false));

        driverController.back().onTrue(swerve.zeroGyroscopeCommand());

        //Autocollect here

        driverController.leftBumper()
                .onTrue(swerve.robotOrientedCommand(true))
                .onFalse(swerve.robotOrientedCommand(false));

        driverController.rightTrigger(0.1).onTrue(
            new ScoreCommand(shooter, elevator, intake)
        );

        driverController.start().whileTrue(leds.partyCommand());

        driverController.y().whileTrue(
            snapToCommand(Rotation2d.fromDegrees(
                DriverStation.getAlliance().get() == Alliance.Red ? 330 : 30
            ))
        );

        //Force-shoot here

        driverController.b().onTrue(
            new OverrideEverythingCommand(new StowCommand(shooter, elevator, intake))
        );

        driverController.pov(0).whileTrue(
            snapToCommand(Rotation2d.fromDegrees(0))
        );

        driverController.pov(90).whileTrue(
            snapToCommand(Rotation2d.fromDegrees(90))
        );

        driverController.pov(180).whileTrue(
            snapToCommand(Rotation2d.fromDegrees(180))
        );

        driverController.pov(270).whileTrue(
            snapToCommand(Rotation2d.fromDegrees(270))
        );



        //Passthrough here

        operatorController.rightBumper().onTrue(
                new PrimeCommand(() -> poseVision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS),
                shooter, elevator, intake)
        );

        operatorController.b().onTrue(
            new PrimeCommand(RangeTable.getPodium(), shooter, elevator, intake)
        );

        operatorController.leftBumper().onTrue(
            new OutakeCommand(shooter, intake)
        );

        operatorController.leftTrigger(0.1).onTrue(
            new IntakeCommand(shooter, elevator, intake)
        );

        operatorController.a().onTrue(
            new OverrideEverythingCommand(new StowCommand(shooter, elevator, intake))
        );

        operatorController.x().onTrue(
            elevator.setStaticPositionCommand(
                ELEVATOR.PIVOT_ANGLE_AMP, ELEVATOR.EXTENSION_METERS_AMP
            )
        );

        operatorController.y().onTrue(
            new ClimbCommand(elevator, intake, shooter)
        );

        operatorController.pov(0).whileTrue(
            elevator.incrementElevatorPositionCommand(0, ELEVATOR.MANUAL_EXTENSION_SPEED)
        );

        operatorController.pov(180).whileTrue(
            elevator.incrementElevatorPositionCommand(0, -ELEVATOR.MANUAL_EXTENSION_SPEED)
        );

        operatorController.back().whileTrue(leds.blinkCommand(LEDS.YELLOW, 2));

        operatorController.start().onTrue(
            new PrimeCommand(RangeTable.getTrap(), shooter, elevator, intake)
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
        return swerve.autonomousInit();
    }

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
