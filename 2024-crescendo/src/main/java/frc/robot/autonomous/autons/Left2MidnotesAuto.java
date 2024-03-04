package frc.robot.autonomous.autons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.*;
import frc.robot.Constants.*;
import frc.robot.autonomous.*;
import frc.robot.commands.*;

public class Left2MidnotesAuto extends BaseAuto{
    private TrajectoryConfig config = new TrajectoryConfig(2,1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory midNote1 = AutonomousPositions.generate(config.setEndVelocity(4),
            AutonomousPositions.SUBWOOFER_DOWN.getPose(),
            AutonomousPositions.MID_NOTE_5.translate(-2,0));

    private SwerveTrajectory mn1ToShoot = AutonomousPositions.generate(config.setEndVelocity(0).setStartVelocity(0),
            AutonomousPositions.MID_NOTE_5.translate(0,0, Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_3.translate(0.5,-1),
            AutonomousPositions.SUBWOOFER_DOWN.translate(1.5,0, Rotation2d.fromDegrees(90)));
    
    private SwerveTrajectory shootToMN2 = AutonomousPositions.generate(config.setEndVelocity(4).setStartVelocity(0),
            AutonomousPositions.SUBWOOFER_DOWN.translate(1.5,0, Rotation2d.fromDegrees(-90)),
            AutonomousPositions.WING_NOTE_3.translate(0.5,-1),
            AutonomousPositions.MID_NOTE_4.translate(-0.75, 0, Rotation2d.fromDegrees(45)));
            
    private SwerveTrajectory mn2ToShoot = AutonomousPositions.generate(config.setEndVelocity(0).setStartVelocity(0),
            AutonomousPositions.MID_NOTE_4.translate(0,0, Rotation2d.fromDegrees(-90)),
            AutonomousPositions.WING_NOTE_3.translate(0.5,-1),
            AutonomousPositions.SUBWOOFER_DOWN.translate(1.5,0, Rotation2d.fromDegrees(90)));

    public void initialize() {
        LimelightTargeting speakerVision = new LimelightTargeting(APRILTAG_LIMELIGHT.LIMELIGHT_NAME, APRILTAG_LIMELIGHT.LOCK_ERROR, APRILTAG_LIMELIGHT.CLOSE_ERROR, APRILTAG_LIMELIGHT.CAMERA_HEIGHT, APRILTAG_LIMELIGHT.CAMERA_ANGLE, APRILTAG_LIMELIGHT.TARGET_HEIGHT);
        PIDController speakerPID = new PIDController(APRILTAG_LIMELIGHT.SPEAKER_TURN_kP, APRILTAG_LIMELIGHT.SPEAKER_TURN_kI, APRILTAG_LIMELIGHT.SPEAKER_TURN_kD);
        speakerPID.setTolerance(APRILTAG_LIMELIGHT.LOCK_ERROR);
        int angle = 0;
        if(DriverStation.getAlliance().get() == Alliance.Red){
            angle = 0;
        }
        queue = new CommandQueue(
                new GyroSetCommand(drive, 0),
                new ShootCommand(shooter, elevator, 3000, 6),
                new FollowerCommand(drive, midNote1),
                new RotateCommand(drive, Rotation2d.fromDegrees(60+angle)),
                // new JointCommand(
                //         new IntakeCommand(intake, shooter),
                //         new AutoCollectCommand(targeting, drive, shooter)
                // ),
                new FollowerCommand(drive, mn1ToShoot),
                new RotateCommand(drive, Rotation2d.fromDegrees(60+angle)),
                // new AutoAimCommand(drive, speakerVision, APRILTAG_LIMELIGHT.LOCK_ERROR),
                // new ShootCommand(shooter, elevator, 4500, 29.5),
                new FollowerCommand(drive, shootToMN2),
                new RotateCommand(drive, Rotation2d.fromDegrees(60+angle)),
                // new JointCommand(
                //         new IntakeCommand(intake, shooter),
                //         new AutoCollectCommand(targeting, drive, shooter)
                // ),
                new FollowerCommand(drive, mn2ToShoot),
                new RotateCommand(drive, Rotation2d.fromDegrees(60+angle))
                // new AutoAimCommand(drive, speakerVision, APRILTAG_LIMELIGHT.LOCK_ERROR),
                // new ShootCommand(shooter, elevator, 4500, 29.5)
                );
    }

    public void periodic() {
        queue.run();
    }
}
