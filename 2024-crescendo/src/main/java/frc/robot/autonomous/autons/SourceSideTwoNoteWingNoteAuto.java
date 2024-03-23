package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Vision;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class SourceSideTwoNoteWingNoteAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.SUBWOOFER_DOWN.getPose(),
            AutonomousPositions.WING_NOTE_3.getPose());

    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(shooter, elevator, 1.4),
                new JointCommand(
                        new FollowerCommand(drive, pathOne.addVision(targeting, -5)),
                        new IntakeCommand(intake, shooter)),
                new RotateCommand(drive, Rotation2d.fromDegrees(-45)),
                new AutoShootCommand(drive, poseVision, elevator, shooter));
    }

    @Override
    public void periodic() {
        queue.run();
    }
}
