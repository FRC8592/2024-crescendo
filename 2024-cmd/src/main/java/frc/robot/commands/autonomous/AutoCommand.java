package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.NewtonCommand;

/**
 * Class to provide subsystems, convenient methods, and a constructor to autonomous commands
 */
public class AutoCommand extends NewtonCommand {
    /**
     * Create an auto routine from the passed-in commands.
     *
     * @param commands as many commands as you want. Will
     * be run in sequence (one after the other).
     */
    protected AutoCommand(Command... commands) {
        super(new SequentialCommandGroup(commands));
    }

    /**
     * Get a choreo trajectory by name as a WPILib trajectory.
     *
     * @param name {@code String}: the name of the .traj file. This shouldn't
     * contain the path or the filename extension (for example, if the file is
     * {@code /home/user/robotcode/src/main/deploy/choreo/MyTrajectory.traj}, the
     * correct {@code String} to put here is {@code "MyTrajectory"})
     *
     * @return The trajectory converted to WPILib's {@link Trajectory}. Throws a
     * {@code FileNotFoundException} if the name doesn't represent a .traj file
     * located in the {@code choreo} folder in the {@code deploy} folder.
     */
    protected static final Trajectory getChoreoTrajectory(String name) {
        // It isn't efficient to import the trajectory again just for a single starting pose,
        // but there's no other easy way to get it. See a few lines down for why that's needed.
        ChoreoTrajectoryState startState = Choreo.getTrajectory(name).getInitialState();

        // Create the PathPlanner trajectory
        PathPlannerTrajectory pathPlannerTraj = (
            PathPlannerPath.fromChoreoTrajectory(name)
            // For some reason, PathPlanner requires a starting ChassisSpeeds and rotation. This
            // is what that Choreo trajectory import above is for.
            .getTrajectory(startState.getChassisSpeeds(), startState.getPose().getRotation())
        );

        List<PathPlannerTrajectory.State> pathPlannerStates = pathPlannerTraj.getStates();
        ArrayList<State> wpilibStates = new ArrayList<>();
        Trajectory wpilibTrajectory;

        // Convert all the PathPlanner states to WPILib trajectory states and add
        // them to the wpilibStates ArrayList
        for (PathPlannerTrajectory.State pathPlannerState : pathPlannerStates) {
            State wpilibState = new State(
                pathPlannerState.timeSeconds,
                pathPlannerState.velocityMps,
                pathPlannerState.accelerationMpsSq,
                pathPlannerState.getTargetHolonomicPose(),
                pathPlannerState.curvatureRadPerMeter
            );
            wpilibStates.add(wpilibState);
        }

        wpilibTrajectory = new Trajectory(wpilibStates);
        return wpilibTrajectory;
    }
}
