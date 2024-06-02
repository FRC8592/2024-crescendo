// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;

public final class AutoManager {
    private static SendableChooser<Command> autoChooser;
    private static ArrayList<Command> autoCommands = new ArrayList<>();

    public static Command getAutonomousCommand(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        return new ParallelCommandGroup(
            swerve.autonomousInitCommand(),
            intake.autonomousInitCommand(),
            elevator.autonomousInitCommand(),
            shooter.autonomousInitCommand(),
            leds.autonomousInitCommand()
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
