package frc.robot.autonomous.autons;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Vision;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.ShootCommand;

public class SourceSideTwoNoteMidNoteAuto extends BaseAuto{
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory pathOne = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0), 
    AutonomousPositions.SUBWOOFER_DOWN.getPose(),
    AutonomousPositions.MID_NOTE_5.getPose()
    );

    private SwerveTrajectory pathTwo = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0), 
    AutonomousPositions.MID_NOTE_5.getPose(),
    AutonomousPositions.WING_NOTE_3.getPose()
    );

    @Override
    public void initialize(){
        queue = new CommandQueue(
          new ShootCommand(shooter, elevator, 1.4),
          new JointCommand(
            new IntakeCommand(intake, shooter),
            new FollowerCommand(drive, pathOne.addVision(targeting, -10))
          ),
          new FollowerCommand(drive, pathTwo.addVision(targeting, -10)),
          new AutoShootCommand(drive, poseVision, elevator, shooter) 
        );
    }
    
    @Override
    public void periodic() {
        queue.run();
    }
}
