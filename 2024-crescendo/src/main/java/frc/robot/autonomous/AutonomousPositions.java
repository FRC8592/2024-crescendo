package frc.robot.autonomous;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// Adjustments made based on simulation results
public enum AutonomousPositions {
    
    
    SUBWOOFER_UP(1.0, 6.25, Rotation2d.fromDegrees(60)),
    SUBWOOFER_MIDDLE(1.4, 5.5, new Rotation2d()),
    SUBWOOFER_DOWN(1.0, 4.5, Rotation2d.fromDegrees(-60)),
    WING_NOTE_1(2.4, 6.5, new Rotation2d()),
    WING_NOTE_2(2.4, 5.5, new Rotation2d()),
    WING_NOTE_3(2.4, 4.125, new Rotation2d()),
    MID_NOTE_1(7.5, 6.85, new Rotation2d()),
    MID_NOTE_2(7.5, 6.125, new Rotation2d()),
    MID_NOTE_3(7.5, 4.125, new Rotation2d()),
    MID_NOTE_4(7.5, 2.6, new Rotation2d()),
    MID_NOTE_5(7.5, 1.1, new Rotation2d()),;

    public static final double FIELD_LENGTH_X = 16.0;
    

    private Translation2d translation;
    private Rotation2d ref;
    AutonomousPositions(double xPos, double yPos, Rotation2d rotation) {
        translation = new Translation2d(xPos, yPos);
        ref = rotation;
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
        boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;

        if (isRed) {
            startPose = new Pose2d(FIELD_LENGTH_X - startPose.getX(), startPose.getY(), Rotation2d.fromDegrees(180 - startPose.getRotation().getDegrees())); // invert, 16 is length of field
            endPose = new Pose2d(FIELD_LENGTH_X - endPose.getX(), endPose.getY(),  Rotation2d.fromDegrees(180 - endPose.getRotation().getDegrees())); // invert, 16 is length of field
        }

        for (int i = 1; i < poses.length - 1; i++) {
            nonEndPoses[i-1] = poses[i].getTranslation();
            if (isRed) {
                nonEndPoses[i-1] = new Translation2d(FIELD_LENGTH_X - nonEndPoses[i-1].getX(), nonEndPoses[i-1].getY());
            }
        }
        System.out.println("PRINTING GENERATED POINTS FOR TRAJECTORY");
        System.out.println(startPose);
        for(int i = 0; i < nonEndPoses.length; i++){
            System.out.println(nonEndPoses[i].toString());
        }
        System.out.println(endPose.toString());

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

    public static SwerveTrajectory generate(TrajectoryConfig setReversed, Pose2d pose, AutonomousPositions wingNote1) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'generate'");
    }
}