package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class AmpSideOneWingNoteAuto extends BaseAuto {
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.SUBWOOFER_UP.getPose(),
            AutonomousPositions.WING_NOTE_1.getPose());

    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(subsystemsManager, 1.4),
                new JointCommand(
                        new FollowerCommand(drive, pathOne.addVision(targeting, -5)),
                        new IntakeCommand(subsystemsManager)),
                // new RotateCommand(drive, Rotation2d.fromDegrees(45)),
                new AutoShootCommand(drive, poseVision, subsystemsManager));
    }

    @Override
    public void periodic() {
        queue.run();
    }

}