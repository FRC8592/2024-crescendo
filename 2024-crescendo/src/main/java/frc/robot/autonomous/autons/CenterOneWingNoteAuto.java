package frc.robot.autonomous.autons;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class CenterOneWingNoteAuto extends BaseAuto {

    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig.setStartVelocity(0).setEndVelocity(0),
         AutonomousPositions.SUBWOOFER_MIDDLE.getPose(),
         AutonomousPositions.WING_NOTE_2.getPose()
    );

    @Override
    public void initialize() {
       queue = new CommandQueue(
          new ShootCommand(subsystemsManager, 1.4).setTag("Score preloaded note..."),
          new JointCommand(
            new IntakeCommand(subsystemsManager),
            new FollowerCommand(drive, pathOne.addVision(targeting, -10))
          ).setTag("Intake while driving to middle wing note..."),
         new AutoShootCommand(drive, poseVision, subsystemsManager)
       );
    }

    @Override
    public void periodic() {
       queue.run();
    }
    
}
