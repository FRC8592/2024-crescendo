package frc.robot.autonomous.autons.WIP;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.*;

public class BlueThreeWingNoteVisionAuto extends BaseAuto {

    // Setting speed and acceleration of each path as shown
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory noteOne = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(),
            AutonomousPositions.SUBWOOFER_MIDDLE.translate(0.3, 0)); // TODO:
                                                                                                             // Optimized
                                                                                                             // for blue
                                                                                                             // now

    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(subsystemsManager, 1.4), // shooting preloaded note
                // First wing note
                new FollowerCommand(drive, noteOne),
                new RotateCommand(drive, Rotation2d.fromDegrees(45)),
                new JointCommand(
                        new AutoCollectCommand(targeting, drive ,
                                shooter),
                        new IntakeCommand(subsystemsManager)),
                new RotateCommand(drive, Rotation2d.fromDegrees(45)),
                new ShootCommand(subsystemsManager, 2.83),

                new RotateCommand(drive, new Rotation2d(-90)), // Turn towards the second note
                new JointCommand(
                        new AutoCollectCommand(targeting, drive,
                                shooter),
                        new IntakeCommand(subsystemsManager)),
                new RotateCommand(drive, Rotation2d.fromDegrees(0)),
                new ShootCommand(subsystemsManager, 2.83),

                new RotateCommand(drive, new Rotation2d(-90)),
                new JointCommand(
                        new AutoCollectCommand(targeting, drive,
                                shooter),
                        new IntakeCommand(subsystemsManager)),
                new RotateCommand(drive, Rotation2d.fromDegrees(-45)),
                new ShootCommand(subsystemsManager, 2.83) // shooting third note
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }

}
