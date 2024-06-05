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
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public final class AutoManager {
    private static SendableChooser<Command> autoChooser;
    private static ArrayList<AutoCommand> autoCommands = new ArrayList<>();

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}.
     * @param swerve
     * @param intake
     * @param elevator
     * @param shooter
     * @param leds
     * @return the command
     */
    public static Command getAutonomousCommand(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        return getAutonomousInitCommand(swerve, intake, elevator, shooter, leds).andThen(autoChooser.getSelected());
    }

    /**
     * Parallel command group that runs all subsystems' autonomous init commands.
     * @param swerve
     * @param intake
     * @param elevator
     * @param shooter
     * @param leds
     * @return the command
     */
    private static Command getAutonomousInitCommand(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        return new ParallelCommandGroup(
            swerve.commands.autonomousInitCommand(),
            intake.commands.autonomousInitCommand(),
            elevator.commands.autonomousInitCommand(),
            shooter.commands.autonomousInitCommand(),
            leds.commands.autonomousInitCommand()
        );
    }

    /**
     * Load (or reload) all autos. This is where programmers should add new autos.
     *
     * @param swerve
     * @param intake
     * @param elevator
     * @param shooter
     * @param leds
     */
    public static void loadAutos(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        autoCommands = new ArrayList<>();
        autoCommands.add(new Testing5Note(swerve, intake, elevator, shooter, leds));
        autoCommands.add(new SystemsCheckAuto(swerve, intake, elevator, shooter, leds));
    }

    /**
     * Create (or re-create) the auto chooser, add all the autos to it
     * ({@link AutoManager#loadAutos(Swerve, Intake, Elevator, Shooter, LEDs)}
     * should ALWAYS be called directly before this), and put it on the dashboard
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
