package frc.robot.autonomous.autons.WIP;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.AutoCollectCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.ShootCommand;

public class RedThreeWingNoteVisionAuto extends BaseAuto {

    // Setting speed and acceleration of each path as shown
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory noteOne = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(),
            AutonomousPositions.SUBWOOFER_MIDDLE.translate(0, 0.3)).addRotation(Rotation2d.fromDegrees(-45)); // TODO: Optimized for blue now
    // An autoCollect command will run here to put us roughly at Wing Note 3
    private SwerveTrajectory shootOne = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_3.translate(0, 0, Rotation2d.fromDegrees(-45))); // TODO: Tune this to shoot into the speaker

    private SwerveTrajectory noteTwo = AutonomousPositions.generate(
            config.setReversed(false).setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_3.translate(0, 0, Rotation2d.fromDegrees(90)));
    // An autoCollect command will run here to put us roughly at Wing Note 2
    private SwerveTrajectory shootTwo = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_2.translate(0, 0, Rotation2d.fromDegrees(0)));// TODO: Tune this to shoot into the speaker

    private SwerveTrajectory noteThree = AutonomousPositions.generate(
            config.setReversed(false).setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(90)));
    // An autoCollect command will run here to put us roughly at Wing Note 1
    private SwerveTrajectory shootThree = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_1.translate(0, 0, Rotation2d.fromDegrees(45))); // TODO: Tune this to shoot into the speaker

    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(subsystemsManager, 1.4), // shooting preloaded note
                // First wing note
                new FollowerCommand(drive, noteOne),
                new JointCommand(
                        new AutoCollectCommand(targeting, drive, shooter),
                        new IntakeCommand(subsystemsManager)),
                new FollowerCommand(drive, shootOne),
                new ShootCommand(subsystemsManager, 2.83),

                new FollowerCommand(drive, noteTwo), // Turn towards the second note
                new JointCommand(
                        new AutoCollectCommand(targeting, drive, shooter),
                        new IntakeCommand(subsystemsManager)),
                new FollowerCommand(drive, shootTwo),
                new ShootCommand(subsystemsManager, 2.83),

                new FollowerCommand(drive, noteThree),
                new JointCommand(
                        new AutoCollectCommand(targeting, drive, shooter),
                        new IntakeCommand(subsystemsManager)),
                new FollowerCommand(drive, shootThree),
                new ShootCommand(subsystemsManager, 2.83) // shooting third note
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }

}
