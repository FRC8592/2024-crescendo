package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.proxies.NewtonCommand;

/**
 * Class to provide subsystems, convenient methods, and a constructor to autonomous commands
 */
public class AutoCommand extends NewtonCommand {
    /**
     * If this is set, the odometry's known position will be set to this at the start of auto
     */
    protected Pose2d startPose = null;

    /**
     * Startup time saving if/when multiple copies of the same path are requested
     */
    private static HashMap<String, Trajectory> cachedChoreoTrajectories = new HashMap<String,Trajectory>();
    private static HashMap<String, Trajectory> cachedPathPlannerTrajectories = new HashMap<String,Trajectory>();

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
     * {@link Commands#none()} as an {@link AutoCommand}
     */
    protected AutoCommand(){
        super(Commands.none());
    }

    /**
     * Get a choreo trajectory by name as a WPILib trajectory.
     *
     * @param name the name of the .traj file; this shouldn't contain the path or
     * the filename extension
     *
     * @return The trajectory converted to WPILib's {@link Trajectory}. Throws a
     * {@code FileNotFoundException} if the name doesn't represent a .traj file
     * located in the {@code choreo} folder in the {@code deploy} folder
     */
    protected static final Trajectory getChoreoTrajectory(String name){
        if(cachedChoreoTrajectories.containsKey(name)){
            return cachedChoreoTrajectories.get(name);
        }
        else{
            Trajectory wpilibTrajectory = fromPathPlannerPath(PathPlannerPath.fromChoreoTrajectory(name));
            cachedChoreoTrajectories.put(name, wpilibTrajectory);
            return wpilibTrajectory;
        }
    }

    /**
     * Get a PathPlanner path by name as a WPILib trajectory.
     *
     * @param name the name of the path in PathPlanner file; this shouldn't contain
     * the file path or the filename extension
     *
     * @return The path converted to WPILib's {@link Trajectory}. Throws a
     * {@code FileNotFoundException} if the name doesn't represent a .path file
     * located in {@code /src/main/deploy/pathplanner/paths}
     */
    protected static final Trajectory getPathPlannerTrajectory(String name){
        if(cachedPathPlannerTrajectories.containsKey(name)){
            return cachedPathPlannerTrajectories.get(name);
        }
        else{
            Trajectory wpilibTrajectory = fromPathPlannerPath(PathPlannerPath.fromPathFile(name));
            cachedPathPlannerTrajectories.put(name, wpilibTrajectory);
            return wpilibTrajectory;
        }
    }

    /**
     * Set the start pose of this auto to the first pose of a Choreo path.
     *
     * @param name the name of the Choreo path to get the start pose from
     */
    protected void setStartStateFromChoreoTrajectory(String name){
        if(!cachedChoreoTrajectories.containsKey(name)){
            getChoreoTrajectory(name); // Adds the path to the cached trajectory map
        }
        this.startPose = cachedChoreoTrajectories.get(name).getInitialPose();
    }

    /**
     * Set the start pose of this auto to the first pose of a PathPlanner path.
     *
     * @param name the name of the PathPlanner path to get the start pose from
     */
    protected void setStartStateFromPathPlannerTrajectory(String name){
        if(cachedPathPlannerTrajectories.containsKey(name)){
            getPathPlannerTrajectory(name);
        }
        this.startPose = cachedPathPlannerTrajectories.get(name).getInitialPose();
    }

    /**
     * Convert a PathPlanner path into a WPILib trajectory
     *
     * @param path the PathPlannerPath to convert
     * @return the path converted to a WPILib trajectory
     */
    private static Trajectory fromPathPlannerPath(PathPlannerPath path){
        PathPlannerTrajectory pathPlannerTraj = (
            path.getTrajectory(new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation())
        );

        List<PathPlannerTrajectory.State> pathPlannerStates = pathPlannerTraj.getStates();
        ArrayList<State> wpilibStates = new ArrayList<>();

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
        return new Trajectory(wpilibStates);
    }
}
