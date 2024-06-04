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
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.leds.LEDCommands;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

public final class AutoManager {
    private static SendableChooser<Command> autoChooser;
    private static ArrayList<Command> autoCommands = new ArrayList<>();

    public static Command getAutonomousCommand(
        Swerve swerve,
        Intake intake,
        Elevator elevator,
        Shooter shooter,
        LEDs leds
    ){
        return new ParallelCommandGroup(
            swerve.commands.autonomousInitCommand(),
            intake.commands.autonomousInitCommand(),
            elevator.commands.autonomousInitCommand(),
            shooter.commands.autonomousInitCommand(),
            leds.commands.autonomousInitCommand()
        ).andThen(autoChooser.getSelected());
    }

    public static void loadAutos(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        autoCommands = new ArrayList<>();
        autoCommands.add(new Testing5Note(swerve, intake, elevator, shooter, leds));
        autoCommands.add(new SystemsCheckAuto(swerve, intake, elevator, shooter, leds));
    }

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
