package frc.robot.autonomous.autons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.*;
import frc.robot.Constants.*;

public class DriveToPointTestAuto extends BaseAuto {
    
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    @Override
    public void initialize() {
        LimelightTargeting speakerVision = new LimelightTargeting(APRILTAG_LIMELIGHT.LIMELIGHT_NAME, APRILTAG_LIMELIGHT.LOCK_ERROR, APRILTAG_LIMELIGHT.CLOSE_ERROR, APRILTAG_LIMELIGHT.CAMERA_HEIGHT, APRILTAG_LIMELIGHT.CAMERA_ANGLE, APRILTAG_LIMELIGHT.TARGET_HEIGHT);
        PIDController speakerPID = new PIDController(APRILTAG_LIMELIGHT.SPEAKER_TURN_kP, APRILTAG_LIMELIGHT.SPEAKER_TURN_kI, APRILTAG_LIMELIGHT.SPEAKER_TURN_kD);
        speakerPID.setTolerance(APRILTAG_LIMELIGHT.LOCK_ERROR);

        queue = new CommandQueue(
                new ShootCommand(shooter, elevator, 3000, 5),
                new JointCommand(
                        new IntakeCommand(intake, shooter),
                        new AutoCollectCommand(targeting, drive,
                                shooter)),
                new DriveToPointCommand(drive, AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), config),
                new ShootCommand(shooter, elevator, 3000,5)
                );
    }

    public void periodic() {
        queue.run();
    }
}
