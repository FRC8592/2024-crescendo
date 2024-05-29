// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(Swerve swerve, Shooter shooter, Elevator elevator, Intake intake, LEDs leds) {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Final");
        PathPlannerTrajectory traj = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(0));
        List<PathPlannerTrajectory.State> states = traj.getStates();
        ArrayList<State> states2 = new ArrayList<>();
        for(PathPlannerTrajectory.State state : states){
            State state2 = new State(
                state.timeSeconds,
                state.velocityMps,
                state.accelerationMpsSq,
                new Pose2d(state.positionMeters, state.targetHolonomicRotation),
                state.curvatureRadPerMeter
            );
            states2.add(state2);
        }
        Trajectory traj2 = new Trajectory(states2);
        return swerve.followPathCommand(traj2);
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
