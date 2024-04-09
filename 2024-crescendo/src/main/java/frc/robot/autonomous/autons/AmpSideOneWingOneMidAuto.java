package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class AmpSideOneWingOneMidAuto extends BaseAuto { // this is amp side one wing one mid
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 3);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.SUBWOOFER_UP.getPose(),
            AutonomousPositions.WING_NOTE_1.getPose()
    ).addRotation(Rotation2d.fromDegrees(0), 0.2);
    private SwerveTrajectory pathTwo = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(false),
            AutonomousPositions.WING_NOTE_1.getPose(),
            AutonomousPositions.MID_NOTE_1.getPose()
    );
    private SwerveTrajectory pathThree = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(true),
            AutonomousPositions.MID_NOTE_1.getPose(),
            AutonomousPositions.WING_NOTE_2.translate(1, 0.5)
    ).addRotation(Rotation2d.fromDegrees(30), 0.2);

    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(subsystemsManager, 1.4),
                // new AutoShootCommand(drive, poseVision, elevator, shooter),
                new JointCommand(
                        new FollowerCommand(drive, pathOne.addVision(targeting, 0)),
                        new IntakeCommand(subsystemsManager)
                ),
                new AutoShootCommand(drive, poseVision, subsystemsManager),
                new JointCommand(
                        new FollowerCommand(drive, pathTwo.addVision(targeting, 10)),
                        new IntakeCommand(subsystemsManager)
                ),
                new FollowerCommand(drive, pathThree),
                new AutoShootCommand(drive, poseVision, subsystemsManager)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }

}