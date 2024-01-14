package frc.robot.autonomous.autons;
import frc.robot.*;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
public class TestAuto extends frc.robot.autonomous.BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(3.75, 1.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(0.5, 1.0);
    private SwerveTrajectory CROSS_DEN_LINE = generate(
        slowConfig
            .setStartVelocity(0)
            .setEndVelocity(0)
            .setReversed(false),
        new Pose2d(0,0,new Rotation2d(0)), new Pose2d(3,0,new Rotation2d(0))
        );
    public void initialize(){
        queue = new CommandQueue(
            new FollowerCommand(swerve, CROSS_DEN_LINE)
        );
    }
    public void periodic() {
        queue.run();
    }
}
