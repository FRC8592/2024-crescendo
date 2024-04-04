package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class AmpSideOneWingTwoMid extends BaseAuto{

    private TrajectoryConfig slowConfig = new TrajectoryConfig(4, 3);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
    AutonomousPositions.SUBWOOFER_UP.getPose(),
    AutonomousPositions.WING_NOTE_1.getPose()
    );
    private SwerveTrajectory pathTwo = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
     AutonomousPositions.WING_NOTE_1.getPose(),
     AutonomousPositions.MID_NOTE_1.translate(-1, 0)
    );
    private SwerveTrajectory pathThree = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(true),
     AutonomousPositions.MID_NOTE_1.translate(-1, 0),
     AutonomousPositions.WING_NOTE_1.translate(2, 0) // stage position
    ).addRotation(Rotation2d.fromDegrees(15));
    
    private SwerveTrajectory pathFour = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(true),
     AutonomousPositions.WING_NOTE_1.translate(2, 0), // stage position
     AutonomousPositions.MID_NOTE_2.translate(-1, 0)
    ).addRotation(Rotation2d.fromDegrees(-30));

    private SwerveTrajectory pathFive = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(true),
    AutonomousPositions.MID_NOTE_2.translate(-1, 0),
    AutonomousPositions.WING_NOTE_1.translate(3, 0),
    AutonomousPositions.WING_NOTE_1.translate(2, 0) // stage position
   ).addRotation(Rotation2d.fromDegrees(15));


    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(subsystemsManager, 1.4),
                new JointCommand(
                    new FollowerCommand(drive, pathOne.addVision(targeting, -15)),
                    new IntakeCommand(subsystemsManager)
                ),
                new AutoShootCommand(drive, poseVision, subsystemsManager),
                new JointCommand(
                    new FollowerCommand(drive, pathTwo.addVision(targeting, -15)),
                    new IntakeCommand(subsystemsManager)
                ),
                new FollowerCommand(drive, pathThree),
                new JointCommand(
                    new FollowerCommand(drive, pathFour.addVision(targeting, -15)),
                    new IntakeCommand(subsystemsManager)
                ),
                new FollowerCommand(drive, pathFive),
                new AutoShootCommand(drive, poseVision, subsystemsManager)
       );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}
