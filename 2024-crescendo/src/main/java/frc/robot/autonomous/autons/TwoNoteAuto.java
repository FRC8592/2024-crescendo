package frc.robot.autonomous.autons;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class TwoNoteAuto extends BaseAuto {

    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
         AutonomousPositions.SUBWOOFER_MIDDLE.getPose(),
         AutonomousPositions.WING_NOTE_2.getPose()
    );

    @Override
    public void initialize() {
       queue = new CommandQueue(
          new ShootCommand(shooter, elevator, 3000, 5),
          new JointCommand(
            new IntakeCommand(intake, shooter),
            new FollowerCommand(drive, pathOne.addVision(targeting))
          ),
          new AutoShootCommand(drive, poseVision, elevator, shooter)
       );
    }

    @Override
    public void periodic() {
       queue.run();
    }
    
}
