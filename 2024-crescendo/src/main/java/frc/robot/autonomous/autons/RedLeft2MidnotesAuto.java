package frc.robot.autonomous.autons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.*;
import frc.robot.Constants.*;
import frc.robot.autonomous.*;
import frc.robot.commands.*;

public class RedLeft2MidnotesAuto extends BaseAuto{
    private TrajectoryConfig config = new TrajectoryConfig(3, 2);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory midNote1 = AutonomousPositions.generate(config.setEndVelocity(3),
            AutonomousPositions.SUBWOOFER_DOWN.getPose(),
            AutonomousPositions.MID_NOTE_5.translate(-2,0));

    private SwerveTrajectory mn1ToShoot = AutonomousPositions.generate(config.setEndVelocity(0).setStartVelocity(3),
            AutonomousPositions.MID_NOTE_5.translate(-2,0, new Rotation2d(135)),
            new Pose2d(2.5,2.3, new Rotation2d(90)),
            AutonomousPositions.SUBWOOFER_DOWN.translate(1.5,0.5,new Rotation2d(90)));

    public void initialize() {
        LimelightTargeting speakerVision = new LimelightTargeting(APRILTAG_LIMELIGHT.LIMELIGHT_NAME, APRILTAG_LIMELIGHT.LOCK_ERROR, APRILTAG_LIMELIGHT.CLOSE_ERROR, APRILTAG_LIMELIGHT.CAMERA_HEIGHT, APRILTAG_LIMELIGHT.CAMERA_ANGLE, APRILTAG_LIMELIGHT.TARGET_HEIGHT);
        PIDController speakerPID = new PIDController(APRILTAG_LIMELIGHT.SPEAKER_TURN_kP, APRILTAG_LIMELIGHT.SPEAKER_TURN_kI, APRILTAG_LIMELIGHT.SPEAKER_TURN_kD);
        speakerPID.setTolerance(APRILTAG_LIMELIGHT.LOCK_ERROR);

        queue = new CommandQueue(
                new GyroSetCommand(drive, -45),
                // new ShootCommand(shooter, elevator, 3000, 5),
                new FollowerCommand(drive, midNote1),
                // new JointCommand(
                //         new IntakeCommand(intake, shooter),
                //         new AutoCollectCommand(targeting, drive, shooter)
                // ),
                new DriveToPointCommand(drive, AutonomousPositions.MID_NOTE_5.getPose(), AutonomousPositions.MID_NOTE_5.translate(-2,0), config.setStartVelocity(0).setEndVelocity(3)),
                new FollowerCommand(drive, mn1ToShoot),
                // new AutoAimCommand(drive, speakerVision, APRILTAG_LIMELIGHT.LOCK_ERROR),
                // new ShootCommand(shooter, elevator, 4500, 29.5),
                // Add a midpoint
                new DriveToPointCommand(drive, AutonomousPositions.WING_NOTE_2.getPose(), new Pose2d(), config.setStartVelocity(0).setEndVelocity(3)),
                new DriveToPointCommand(drive, AutonomousPositions.SUBWOOFER_MIDDLE.translate(-1,2), AutonomousPositions.MID_NOTE_4.translate(-2,0), config.setStartVelocity(3).setEndVelocity(0)),
                // new JointCommand(
                //         new IntakeCommand(intake, shooter),
                //         new AutoCollectCommand(targeting, drive, shooter)
                // ),
                new DriveToPointCommand(drive, AutonomousPositions.MID_NOTE_4.getPose(), AutonomousPositions.SUBWOOFER_MIDDLE.translate(1,-1), config.setEndVelocity(1))
                // new AutoAimCommand(drive, speakerVision, APRILTAG_LIMELIGHT.LOCK_ERROR),
                // new ShootCommand(shooter, elevator, 4500, 29.5)
                );
    }

    public void periodic() {
        queue.run();
    }
}