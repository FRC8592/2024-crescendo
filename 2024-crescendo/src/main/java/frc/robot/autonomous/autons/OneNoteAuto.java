package frc.robot.autonomous.autons;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class OneNoteAuto extends BaseAuto{

    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    @Override
    public void initialize() {
       queue = new CommandQueue(
            new ShootCommand(shooter, elevator, 3000, 5),
            new JointCommand(
                new AutoCollectCommand(targeting, drive, shooter),
                new IntakeCommand(intake, shooter).setTimeout(2)
            )
       );
    }

    @Override
    public void periodic() {
       queue.run();
    }
    
}
