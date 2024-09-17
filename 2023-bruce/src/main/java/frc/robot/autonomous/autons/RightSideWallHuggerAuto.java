package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;
import frc.robot.FRCLogger;
import frc.robot.Vision;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;

public class RightSideWallHuggerAuto extends BaseAuto {


    private TrajectoryConfig config = new TrajectoryConfig(4, 3);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 1);

    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setReversed(false).setStartVelocity(0).setEndVelocity(1), 
        AutonomousPositions.SUBWOOFER_MIDDLE.getPose(),
        AutonomousPositions.WING_NOTE_1.getPose()).addRotation(Rotation2d.fromDegrees(-45));

    private SwerveTrajectory dumbPosition = AutonomousPositions.generate(slowConfig.setReversed(false).setStartVelocity(1).setEndVelocity(1),
        AutonomousPositions.WING_NOTE_1.getPose(),
        AutonomousPositions.WING_NOTE_1.translate(2.5, 0));

    private SwerveTrajectory pathTwo = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(1).setEndVelocity(0.5), 
        AutonomousPositions.WING_NOTE_1.translate(2.5, 0), 
        AutonomousPositions.MID_NOTE_1.translate(0, -0.3));

    private SwerveTrajectory shootOne = AutonomousPositions.generate(config.setReversed(true).setStartVelocity(0.5).setEndVelocity(0), 
        AutonomousPositions.MID_NOTE_1.translate(0, -0.3),
        AutonomousPositions.WING_NOTE_1.translate(3.0, -0.26));

    private SwerveTrajectory pathThree = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(0).setEndVelocity(0.5),
        AutonomousPositions.WING_NOTE_1.translate(3.0, -0.2),
        AutonomousPositions.MID_NOTE_2.translate(0, -0.7));

    private SwerveTrajectory shootTwo = AutonomousPositions.generate(config.setReversed(true).setStartVelocity(0.5).setEndVelocity(0),
        AutonomousPositions.MID_NOTE_2.translate(0, -0.7),
        AutonomousPositions.WING_NOTE_1.translate(3.0,-0.26));

    private FRCLogger logger = new FRCLogger(true, "ThreeNoteAutoLog");
    private Vision lockToSpeakerVision = new Vision(Constants.LIMELIGHT_VISION, Constants.DRIVE_TO_LOCK_ERROR,
     Constants.DRIVE_TO_CLOSE_ERROR, Constants.DRIVE_TO_CAMERA_HEIGHT, Constants.DRIVE_TO_CAMERA_ANGLE,
     Constants.DRIVE_TO_TARGET_HEIGHT, logger);

    
    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, pathOne),
            new FollowerCommand(drive, dumbPosition, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new FollowerCommand(drive,pathTwo ),
            new FollowerCommand(drive, shootOne, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new FollowerCommand(drive, pathThree),
            new FollowerCommand(drive, shootTwo, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision)

     );
    }

    @Override
    public void periodic() {
       queue.run();
    }
    
}
