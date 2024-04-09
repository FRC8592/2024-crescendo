package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class AmpSideTwoMidAuto extends BaseAuto{

    private TrajectoryConfig slowConfig = new TrajectoryConfig(4, 2);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
    AutonomousPositions.SUBWOOFER_UP.getPose(),
    AutonomousPositions.WING_NOTE_1.translate(-1, 0.65),
    AutonomousPositions.WING_NOTE_1.translate(0, 0.65),
    AutonomousPositions.WING_NOTE_1.translate(1,0.65),
    AutonomousPositions.WING_NOTE_1.translate(2, 0.2),

     AutonomousPositions.MID_NOTE_1.translate(-0.5, 0)
    );
    private SwerveTrajectory pathTwo = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(true),
     AutonomousPositions.MID_NOTE_1.translate(-0.5, -0.25),
     AutonomousPositions.WING_NOTE_1.translate(3, 0),
     AutonomousPositions.WING_NOTE_2.translate(1.25, 1) // stage position
    ).addRotation(Rotation2d.fromDegrees(15));
    
    private SwerveTrajectory pathThree = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(true),
     AutonomousPositions.WING_NOTE_2.translate(1.25, 1), // stage position
     AutonomousPositions.WING_NOTE_1.translate(3, 0), 
     AutonomousPositions.MID_NOTE_2.translate(-1.25, -1, Rotation2d.fromDegrees(45))
    ).addRotation(Rotation2d.fromDegrees(-30));

    private SwerveTrajectory pathFour = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0).setReversed(false),
    AutonomousPositions.MID_NOTE_2.translate(-1, 0),
    AutonomousPositions.WING_NOTE_1.translate(3, 0),
    AutonomousPositions.WING_NOTE_1.translate(2, 0) // stage position
   ).addRotation(Rotation2d.fromDegrees(15));


    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(subsystemsManager, 1.4),
                new JointCommand(
                    new FollowerCommand(drive, pathOne.addVision(targeting, 0)),
                    new IntakeCommand(subsystemsManager)
                ),
                new JointCommand(
                    new FollowerCommand(drive, pathTwo),
                    new AutoPrimeCommand(subsystemsManager, poseVision)
                ), 
                new AutoShootCommand(drive, poseVision, subsystemsManager),
                new JointCommand(
                    new FollowerCommand(drive, pathThree.addVision(targeting, 0)),
                    new IntakeCommand(subsystemsManager)
               )
             //   new FollowerCommand(drive, pathFive)
                // new AutoShootCommand(drive, poseVision, subsystemsManager)
       );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}
