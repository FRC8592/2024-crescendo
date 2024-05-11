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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
        swerve.setDefaultCommand(swerve.driveCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            () -> driverController.getRightX()
        ));

        leds.setDefaultCommand(leds.defaultCommand());
    }

    private void configureBindings() {
        // Slow Mode is Driver Controller Right Bumper
        // Reset Gyro is Driver Controller Back Button
        // Auto Collect is Driver Controller A Button
        // Robot Oriented is Driver Controller Left Bumper
        // Score is Driver Controller Right Trigger
        // Party Mode is Driver Controller Start Button
        // Pass Aim is Driver Controller Y Button
        // Force Shoot is Driver Controller X Button

        // Passthrough is Operator Controller Right Trigger
        // Range Table Shoot is Operator Controller Right Bumper
        // Shoot From Podium is Operator Controller B Button
        // Outake is Operator Controller Left Bumper
        // Intake is Operator Controller Left Trigger
        // Stow is Operator Controller A Button or Driver Controller B Button
        // Amp is Operator Controller X Button
        // climb is Operator Controller Y Button
        // Manual Extend is Operator Controller DPAD Up
        // Manual Retract is Operator Controller DPAD Down
        // Led Amp Signal is Operator Controller Back Button
        // Trap Prime is Operator Controller Start Button

        operatorController.a().onTrue(new OverrideEverythingCommand(new StowCommand(shooter, elevator, intake)));
        operatorController.rightBumper().onTrue(
                new PrimeCommand(() -> poseVision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS),
                shooter, elevator, intake)
        );
        driverController.rightTrigger(0.1).onTrue(shooter.fireCommand().onlyIf(() -> shooter.readyToShoot()));
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
}
