package frc.robot.autonomous.autons;

import javax.security.auth.kerberos.DelegationPermission;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Vision;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.FRCLogger;

import frc.robot.commands.FollowerCommand;
import frc.robot.Constants;

public class SixNoteAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(3, 2);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 1);
    
        private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig, 
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), 
            AutonomousPositions.WING_NOTE_3.translate(0.2, 0.5, Rotation2d.fromDegrees(-45)));

        private SwerveTrajectory pathTwo = AutonomousPositions.generate(slowConfig.setReversed(false).setEndVelocity(0), 
            AutonomousPositions.WING_NOTE_3.translate(0.2, 0.5, Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(0))).addRotation(Rotation2d.fromDegrees(0));

        private SwerveTrajectory pathThree = AutonomousPositions.generate(slowConfig.setReversed(false).setStartVelocity(0).setEndVelocity(1), 
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_1.getPose());
        
        private SwerveTrajectory dumbPosition = AutonomousPositions.generate(slowConfig.setReversed(false).setStartVelocity(1).setEndVelocity(1),
                AutonomousPositions.WING_NOTE_1.getPose(),
                AutonomousPositions.WING_NOTE_1.translate(2.5, 0));


        private SwerveTrajectory pathFour = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(1).setEndVelocity(0), 
            AutonomousPositions.WING_NOTE_1.translate(2.5, 0), 
            AutonomousPositions.MID_NOTE_1.translate(0, -0.3));

        private SwerveTrajectory pathFive = AutonomousPositions.generate(config.setReversed(true).setStartVelocity(0), 
            AutonomousPositions.MID_NOTE_1.translate(0, -0.3),
            AutonomousPositions.WING_NOTE_1.translate(3.0, -0.2));

    private SwerveTrajectory pathSix = AutonomousPositions.generate(config.setReversed(false),
            AutonomousPositions.WING_NOTE_1.translate(3.0, -0.2),
            AutonomousPositions.MID_NOTE_2.translate(0, -0.5));

    private SwerveTrajectory pathSeven = AutonomousPositions.generate(config.setReversed(true),
            AutonomousPositions.MID_NOTE_2.translate(0, -0.5),
            AutonomousPositions.WING_NOTE_1.translate(3.0,0.2));

    private FRCLogger logger = new FRCLogger(true, "ThreeNoteAutoLog");
    private Vision lockToSpeakerVision = new Vision(Constants.LIMELIGHT_VISION, Constants.DRIVE_TO_LOCK_ERROR,
     Constants.DRIVE_TO_CLOSE_ERROR, Constants.DRIVE_TO_CAMERA_HEIGHT, Constants.DRIVE_TO_CAMERA_ANGLE,
     Constants.DRIVE_TO_TARGET_HEIGHT, logger);


    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, pathOne,  AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new FollowerCommand(drive,pathTwo,  AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new FollowerCommand(drive, pathThree,  AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new FollowerCommand(drive, dumbPosition,  AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new FollowerCommand(drive, pathFour,  AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new FollowerCommand(drive, pathFive,  AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new DelayCommand(0.5),
            new FollowerCommand(drive, pathSix,  AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new DelayCommand(0.1),
            new FollowerCommand(drive, pathSeven,  AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose(),lockToSpeakerVision),
            new DelayCommand(0.5)

        );
        // TODO Auto-generated method stub
        
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        queue.run();
        
    }
    
}