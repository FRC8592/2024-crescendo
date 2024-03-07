package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.*;
import frc.robot.autonomous.*;
public class DriveToPointCommand extends Command {
    private Swerve drive;
    private SwerveTrajectory trajectory;
    private FollowerCommand followerCommand;
    private Pose2d targetPose;
    private TrajectoryConfig config;
    public DriveToPointCommand(Swerve drive, Pose2d targetPose, TrajectoryConfig config){
        this.drive = drive;
        this.targetPose = targetPose;
        this.config = config;
    }
    public DriveToPointCommand(Swerve drive, Pose2d startPose, Pose2d targetPose, TrajectoryConfig config){
        this.drive = drive;
        Pose2d currentPose = startPose; // Start by creating this so we don't call drive.getCurrentPos() a bunch of times
        Pose2d modifiedStartPose = new Pose2d(currentPose.getTranslation(), 
                Rotation2d.fromRadians(Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()))); // The robot will go straight to the target point without any fancy curves
        Pose2d modifiedEndPose = new Pose2d(targetPose.getTranslation(), 
                Rotation2d.fromRadians(Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX())));
        this.trajectory = AutonomousPositions.generate(config, modifiedStartPose, modifiedEndPose);
        this.followerCommand = new FollowerCommand(drive, trajectory);
    }
    public void initialize() {
        if(followerCommand == null){
            Pose2d currentPose = drive.getCurrentPos(); // Start by creating this so we don't call drive.getCurrentPos() a bunch of times
            Pose2d modifiedStartPose = new Pose2d(currentPose.getTranslation(), 
                    Rotation2d.fromRadians(Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()))); // The robot will go straight to the target point without any fancy curves
            Pose2d modifiedEndPose = new Pose2d(targetPose.getTranslation(), 
                    Rotation2d.fromRadians(Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX())));
            this.trajectory = AutonomousPositions.generate(config, modifiedStartPose, modifiedEndPose);
            this.followerCommand = new FollowerCommand(drive, trajectory);
        }
        this.followerCommand.initialize();
    }
    public boolean execute() {
        return this.followerCommand.execute();
    }
    public void shutdown() {
        this.followerCommand.shutdown();
    }
    
}
