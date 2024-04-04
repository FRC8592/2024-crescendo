package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class SourceSideTwoMidAuto extends BaseAuto{ //TODO THIS IS SOURCE SIDE (RENAME TO SOURCE SIDE 2 MID NOTES)

    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 3);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(false),
    AutonomousPositions.SUBWOOFER_DOWN.getPose(),
    AutonomousPositions.MID_NOTE_5.translate(0, -.2)
    ).addRotation(new Rotation2d(), 0.5);
    private SwerveTrajectory pathTwo = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(1).setReversed(true),
     AutonomousPositions.MID_NOTE_5.translate(0, -.2),
     AutonomousPositions.MID_NOTE_5.translate(-3, 0.5),
     AutonomousPositions.WING_NOTE_2.translate(1, -2.6)
    ).addRotation(Rotation2d.fromDegrees(-60));
    private SwerveTrajectory pathThree = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(false),
     AutonomousPositions.WING_NOTE_2.translate(1, -2.6),
     AutonomousPositions.MID_NOTE_3.translate(-3, -0.5),
     AutonomousPositions.MID_NOTE_4.translate(0, -.5)
    ).addRotation(Rotation2d.fromDegrees(-15), 0.2);
    private SwerveTrajectory pathFour = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(true),
     AutonomousPositions.MID_NOTE_4.translate(0, 0.2),
     AutonomousPositions.MID_NOTE_3.translate(-2, 0),
     AutonomousPositions.WING_NOTE_2.translate(0.5, -2.25)
    ).addRotation(Rotation2d.fromDegrees(-60), 0.4);

    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(subsystemsManager, 1.4),
                new JointCommand(
                    new FollowerCommand(drive, pathOne.addVision(targeting, -1)),
                    new IntakeCommand(subsystemsManager)
                ),
                new FollowerCommand(drive, pathTwo),
                new DelayCommand(0.1),
                new AutoShootCommand(drive, poseVision, subsystemsManager),
                // new DelayCommand(0.1),
                new JointCommand(
                    new FollowerCommand(drive, pathThree.addVision(targeting, 25)),
                    new IntakeCommand(subsystemsManager)
                ),
                new DelayCommand(0.1),
                new FollowerCommand(drive, pathFour),
                new DelayCommand(0.1),
                new AutoShootCommand(drive, poseVision, subsystemsManager)
       );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}