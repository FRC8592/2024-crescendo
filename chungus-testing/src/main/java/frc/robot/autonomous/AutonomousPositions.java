package frc.robot.autonomous;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

// Adjustments made based on simulation results
public abstract class AutonomousPositions {
    private Translation2d translation;
    private Rotation2d ref;
    AutonomousPositions(double xPos, double yPos) {
        translation = new Translation2d(xPos, yPos);
        ref = new Rotation2d();
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public Pose2d getPose() {
        return new Pose2d(translation, ref);
    }

    public Pose2d translate(double dx, double dy) {
        return new Pose2d(getPose().getX() + dx, getPose().getY() + dy, getPose().getRotation());
    }

    public Pose2d rotate(Rotation2d newRotation) {
        return new Pose2d(getTranslation(), getPose().getRotation().plus(newRotation));
    }

    public Pose2d translate(double dx, double dy, Rotation2d newRotation) {
        return new Pose2d(getPose().getX() + dx, getPose().getY() + dy, newRotation);
    }

    public static SwerveTrajectory generate(TrajectoryConfig config, Pose2d ... poses) {
        Trajectory traj = new Trajectory();
        Pose2d startPose = poses[0];
        Pose2d endPose = poses[poses.length - 1];
        Translation2d[] nonEndPoses = new Translation2d[poses.length - 2];
        for (int i = 1; i < poses.length - 1; i++) {
            nonEndPoses[i-1] = poses[i].getTranslation();
        }

        traj = TrajectoryGenerator.generateTrajectory(
            startPose, 
            List.of(nonEndPoses), 
            endPose, 
            config
        );

        return new SwerveTrajectory(traj).setTrajectoryConfiguration(config);
    }

    public static SwerveTrajectory generate(TrajectoryConfig config, Rotation2d rotation, double turnDelay, Pose2d ... poses) {
        Trajectory traj = new Trajectory();
        Pose2d startPose = poses[0];
        Pose2d endPose = poses[poses.length - 1];
        Translation2d[] nonEndPoses = new Translation2d[poses.length - 2];
        for (int i = 1; i < poses.length - 1; i++) {
            nonEndPoses[i-1] = poses[i].getTranslation();
        }

        traj = TrajectoryGenerator.generateTrajectory(
            startPose, 
            List.of(nonEndPoses), 
            endPose, 
            config
        );

        return new SwerveTrajectory(traj).setTrajectoryConfiguration(config).addRotation(rotation, 2 * Math.PI, turnDelay);
    }
}