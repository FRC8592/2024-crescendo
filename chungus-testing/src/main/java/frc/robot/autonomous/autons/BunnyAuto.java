package frc.robot.autonomous.autons;

import frc.robot.*;
import frc.robot.autonomous.*;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;
import static frc.robot.autonomous.AutonomousPositions.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class BunnyAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(3.75, 1.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2.0, 0.6);
    private SwerveTrajectory GO_TO_BUNNY_DROPOFF = generate(
            slowConfig //TODO: Go back to fast config once tested
                    .setStartVelocity(0)
                    .setEndVelocity(1)
                    .setReversed(false),
            Constants.LOCATION_START,
            Constants.LOCATION_OUTSIDE_DEN,
            Constants.LOCATION_AVOID_BARRIER,
            Constants.LOCATION_BUNNY_DROPOFF
            );
    private SwerveTrajectory BACK_TO_START = generate(
            slowConfig //TODO: Go back to fast config once tested
                    .setStartVelocity(0)
                    .setEndVelocity(1)
                    .setReversed(false),
            Constants.LOCATION_AVOID_BARRIER,
            Constants.LOCATION_OUTSIDE_DEN);
    public void initialize() {
        queue = new CommandQueue(
            new JointCommand(
                new FollowerCommand(swerve, GO_TO_BUNNY_DROPOFF)
                // new BunnyHoldCommand(dropper)
            ),
            new BunnyDropperCommand(dropper),
            new DelayCommand(1)
            // new FollowerCommand(swerve, BACK_TO_START)
        );
    }

    public void periodic() {
        queue.run();
    }
}
