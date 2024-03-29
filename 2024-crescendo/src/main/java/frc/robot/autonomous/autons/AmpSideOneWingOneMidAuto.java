package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class AmpSideOneWingOneMidAuto extends BaseAuto{ //NOTE THIS IS SOURCE SIDE

    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 3);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
    AutonomousPositions.SUBWOOFER_DOWN.getPose(),
    AutonomousPositions.MID_NOTE_5.getPose()
    ).addRotation(new Rotation2d(), 0.5);
    private SwerveTrajectory pathTwo = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
     AutonomousPositions.MID_NOTE_5.getPose(),
     AutonomousPositions.WING_NOTE_2.translate(0.5, -2.25)
    ).addRotation(Rotation2d.fromDegrees(-45));
    private SwerveTrajectory pathThree = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(true),
     AutonomousPositions.MID_NOTE_1.translate(2, 0),
     AutonomousPositions.WING_NOTE_1.translate(2, 0) // stage position
    ).addRotation(Rotation2d.fromDegrees(15));

    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(shooter, elevator, 1.4),
                new JointCommand(
                    new FollowerCommand(drive, pathOne.addVision(targeting, -1)),
                    new IntakeCommand(intake, shooter)
                ),
                new FollowerCommand(drive, pathTwo),
                new DelayCommand(0.1),
                new AutoShootCommand(drive, poseVision, elevator, shooter)
                // new JointCommand(
                //     new FollowerCommand(drive, pathThree.addVision(targeting, -5)),
                //     new IntakeCommand(intake, shooter)
                // ),
                // new FollowerCommand(drive, pathThree),
                // new DelayCommand(0.1),
                // new AutoShootCommand(drive, poseVision, elevator, shooter)
       );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}
