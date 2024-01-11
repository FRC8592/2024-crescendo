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

public class BerryAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(3.75, 1.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1.0, 1.0);
    private SwerveTrajectory GO_TO_SHOOT_POSITION = generate(
            slowConfig //This will be joined with the shooter spinning up, and the slow config will ensure it has time to get up to speed.
                    .setStartVelocity(0)
                    .setEndVelocity(0)
                    .setReversed(false),
            Constants.LOCATION_START,
            Constants.LOCATION_BERRY_DROPOFF);
    private SwerveTrajectory CROSS_DEN_LINE = generate(
            slowConfig
                    .setStartVelocity(0)
                    .setEndVelocity(0)
                    .setReversed(false),
            Constants.LOCATION_OUTSIDE_DEN,
            Constants.LOCATION_OUTSIDE_DEN.transformBy(new Transform2d(
                    new Translation2d(0, 2),
                    new Rotation2d(0))));
    public void initialize(){
        queue = new CommandQueue(
            new JointCommand(
                new FollowerCommand(swerve, vision, GO_TO_SHOOT_POSITION),
                new ShooterSpinCommand(shooter)
            ),
            new ShootCommand(shooter, intake),
            new FollowerCommand(swerve, CROSS_DEN_LINE)
        );
    }
    public void periodic(){
        queue.run();
    }
}
