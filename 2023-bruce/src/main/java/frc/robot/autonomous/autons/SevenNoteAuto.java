package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;
import frc.robot.FRCLogger;
import frc.robot.Vision;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;

public class SevenNoteAuto extends BaseAuto{
    private TrajectoryConfig config = new TrajectoryConfig(3, 2);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 2);
    
    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig, 
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), 
            AutonomousPositions.WING_NOTE_3.translate(0.2, 0.5, Rotation2d.fromDegrees(-45)));

    private SwerveTrajectory pathTwo = AutonomousPositions.generate(slowConfig.setReversed(false).setEndVelocity(0), 
            AutonomousPositions.WING_NOTE_3.translate(0.2, 0.5, Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(0))).addRotation(Rotation2d.fromDegrees(0));

    private SwerveTrajectory pathThree = AutonomousPositions.generate(slowConfig.setReversed(true).setEndVelocity(1), 
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(0)),
            AutonomousPositions.WING_NOTE_1.getPose(),
            AutonomousPositions.WING_NOTE_1.translate(1.0,0.2,Rotation2d.fromDegrees(180)));
        

    private SwerveTrajectory pathFour = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(1).setEndVelocity(1.5), 
            AutonomousPositions.WING_NOTE_1.translate(1.0, 0.2), 
            AutonomousPositions.MID_NOTE_1.translate(0, 0.3),
            AutonomousPositions.MID_NOTE_1.translate(-1, -0.1, Rotation2d.fromDegrees(180)));

    private SwerveTrajectory pathFive = AutonomousPositions.generate(config.setReversed(true).setStartVelocity(1.5), 
            AutonomousPositions.MID_NOTE_1.translate(-1, -0.1),
            AutonomousPositions.WING_NOTE_1.translate(3.0, -0.2));

    private SwerveTrajectory pathSix = AutonomousPositions.generate(config.setReversed(false),
            AutonomousPositions.WING_NOTE_1.translate(3.0, -0.2),
            AutonomousPositions.MID_NOTE_2.getPose());

    private SwerveTrajectory pathSeven = AutonomousPositions.generate(config.setReversed(true),
            AutonomousPositions.MID_NOTE_2.getPose(),
            AutonomousPositions.WING_NOTE_1.translate(3.0,0.2));

    private SwerveTrajectory pathEight = AutonomousPositions.generate(config.setReversed(false), 
            AutonomousPositions.WING_NOTE_1.translate(3.0, 0.2),
            AutonomousPositions.MID_NOTE_3.getPose());

    private SwerveTrajectory pathNine = AutonomousPositions.generate(config.setReversed(true), 
            AutonomousPositions.MID_NOTE_3.getPose(),
            AutonomousPositions.MID_NOTE_3.translate(-4, -0.7)
    );

    private FRCLogger logger = new FRCLogger(true, "ThreeNoteAutoLog");
    private Vision lockToSpeakerVision = new Vision(Constants.LIMELIGHT_VISION, Constants.DRIVE_TO_LOCK_ERROR,
     Constants.DRIVE_TO_CLOSE_ERROR, Constants.DRIVE_TO_CAMERA_HEIGHT, Constants.DRIVE_TO_CAMERA_ANGLE,
     Constants.DRIVE_TO_TARGET_HEIGHT, logger);


    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, pathOne),
            new FollowerCommand(drive,pathTwo),
            new FollowerCommand(drive, pathThree),
            new FollowerCommand(drive, pathFour),
            new FollowerCommand(drive, pathFive),
            new DelayCommand(0.5),
            new FollowerCommand(drive, pathSix),
            new DelayCommand(0.1),
            new FollowerCommand(drive, pathSeven),
            new DelayCommand(0.5),
            new FollowerCommand(drive, pathEight),
            new FollowerCommand(drive, pathNine)
        );
    }

    @Override
    public void periodic() {
       queue.run();
    }
    
}
