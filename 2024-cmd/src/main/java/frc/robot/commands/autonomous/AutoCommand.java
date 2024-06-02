package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class AutoCommand extends WrapperCommand {
    protected static BooleanSupplier flipPathToRedSide = (
        () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
    );
    public AutoCommand(Command... commands) {
        super(new SequentialCommandGroup(commands));
    }

    protected static Trajectory getChoreoTrajectory(String name) {
        PathPlannerTrajectory pathPlannerTraj = (
            PathPlannerPath.fromChoreoTrajectory(name)
            .getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(0))
        );

        List<PathPlannerTrajectory.State> pathPlannerStates = pathPlannerTraj.getStates();
        ArrayList<State> wpilibStates = new ArrayList<>();
        Trajectory wpilibTrajectory;
        for (PathPlannerTrajectory.State pathPlannerState : pathPlannerStates) {
            State wpilibState = new State(
                    pathPlannerState.timeSeconds,
                    pathPlannerState.velocityMps,
                    pathPlannerState.accelerationMpsSq,
                    pathPlannerState.getTargetHolonomicPose(),
                    pathPlannerState.curvatureRadPerMeter);
            wpilibStates.add(wpilibState);
        }
        wpilibTrajectory = new Trajectory(wpilibStates);
        return wpilibTrajectory;
    }
}
