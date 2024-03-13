package frc.robot.autonomous.autons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Swerve;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;
import frc.robot.LimelightTargeting;

public class TwoNoteVisionAuto extends BaseAuto {

    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory shootTwo = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(),
            AutonomousPositions.WING_NOTE_2.getPose(),
            AutonomousPositions.WING_NOTE_2.getPose());

    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(shooter, elevator, 3000, 5),
                new JointCommand(
                        new IntakeCommand(intake, shooter),
                        new AutoCollectCommand(targeting, drive,
                                shooter)),
                new AutoShootCommand(drive, poseVision, elevator, shooter)
                );
    }

    @Override
    public void periodic() {
        queue.run();
    }

}
