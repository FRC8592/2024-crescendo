// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.autons.*;
import frc.robot.commands.proxies.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * General class for autonomous management (loading autos, sending the chooser, getting the
 * user-selected auto command, etc).
 */
public final class AutoManager {
    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static LEDs leds = LEDs.getInstance();

    private static SendableChooser<AutoCommand> autoChooser;
    private static ArrayList<AutoCommand> autoCommands = new ArrayList<>();

    /**
     * Load all autos and broadcast the chooser.
     *<p>
     * * This is where programmers should add new autos.
     *
     * @apiNote This should be called on {@link Robot#robotInit()} only;
     * this function will have relatively long delays due to loading paths.
     */
    public static void prepare(){
        autoCommands = new ArrayList<>();

        autoCommands.add(new PreloadThreeWingNoteAuto());
        autoCommands.add(new Testing5Note());
        autoCommands.add(new SystemsCheckAuto());


        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("DEFAULT - No auto", new AutoCommand());
        for(AutoCommand c : autoCommands){
            autoChooser.addOption(
                c.getClass().getSimpleName()+(
                    c.startPose == null ? " (WARNING: NO START POSE)" : ""
                ), c
            );
        }
        Shuffleboard.getTab("Autonomous Config").add(autoChooser);
    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand(){
        AutoCommand autoCommand = autoChooser.getSelected();

        if(autoCommand.startPose == null){ // If we have no start pose, just run the auto
            return getAutonomousInitCommand().andThen(
                // If we don't keep this command from registering as composed,
                // the code will crash if we try to run an auto twice without
                // restarting robot code.
                new MultiComposableCommand(autoCommand)
            );
        }
        else{ // If we do have a starting pose, reset the odometry to that first
            return getAutonomousInitCommand().andThen(
                swerve.commands.setOdometryPoseCommand(autoCommand.startPose, Suppliers.robotRunningOnRed)
            ).andThen(
                new MultiComposableCommand(autoCommand)
            );
        }
    }

    /**
     * Parallel command group that runs all subsystems' autonomous init commands.
     *
     * @return the command
     */
    private static Command getAutonomousInitCommand(){
        return new ParallelCommandGroup(
            swerve.commands.autonomousInitCommand(),
            intake.commands.autonomousInitCommand(),
            elevator.commands.autonomousInitCommand(),
            shooter.commands.autonomousInitCommand(),
            leds.commands.autonomousInitCommand()
        );
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
