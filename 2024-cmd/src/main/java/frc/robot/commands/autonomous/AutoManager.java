// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
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
    private static Swerve swerve;
    private static Intake intake;
    private static Elevator elevator;
    private static Shooter shooter;
    private static LEDs leds;

    private static SendableChooser<Command> autoChooser;
    private static ArrayList<AutoCommand> autoCommands = new ArrayList<>();

    /**
     * Add all the subsystems to the Auto Manager to avoid the inconvenience of passing
     * them around.
     *
     * @param swerve
     * @param intake
     * @param elevator
     * @param shooter
     * @param leds
     */
    public static void addSubsystems(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        AutoManager.swerve = swerve;
        AutoManager.shooter = shooter;
        AutoManager.intake = intake;
        AutoManager.elevator = elevator;
        AutoManager.leds = leds;
    }
    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}.
     *
     * @return the command
     */
    public static Command getAutonomousCommand(){
        return getAutonomousInitCommand().andThen(
            // If we don't keep this command from registering as composed,
            // the code will crash if we try to run an auto twice without
            // restarting robot code.
            new MultiComposableCommand(autoChooser.getSelected())
        );
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

    /**
     * Load all autos. This is where programmers should add new autos.
     *
     * @apiNote This should be called on {@link Robot#robotInit()} only;
     * this function will have relatively long delays due to loading paths.
     *
     * @apiNote {@link AutoManager#broadcastChooser()} should be called
     * after running this function.
     */
    public static void loadAutos(){
        autoCommands = new ArrayList<>();
        autoCommands.add(new PreloadThreeWingNoteAuto(swerve, intake, elevator, shooter, leds));
        autoCommands.add(new Testing5Note(swerve, intake, elevator, shooter, leds));
        autoCommands.add(new SystemsCheckAuto(swerve, intake, elevator, shooter, leds));
    }

    /**
     * Create the auto chooser, add all the autos to it, and put it on the dashboard
     *
     * @apiNote {@link AutoManager#loadAutos(Swerve, Intake, Elevator, Shooter, LEDs)}
     * should ALWAYS be called directly before this. If it isn't, the chooser will be
     * empty.
     */
    public static void broadcastChooser(){
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("DEFAULT - No auto", Commands.none());
        for(Command c : autoCommands){
            autoChooser.addOption(c.getClass().getSimpleName(), c);
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
