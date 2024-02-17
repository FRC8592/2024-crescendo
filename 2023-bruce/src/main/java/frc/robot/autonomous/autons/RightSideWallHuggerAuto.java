package frc.robot.autonomous.autons;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.commands.CommandQueue;

public class RightSideWallHuggerAuto extends BaseAuto {


    private TrajectoryConfig config = new TrajectoryConfig(2, 2);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 2);

    
    @Override
    public void initialize() {
        queue = new CommandQueue(



        );
    }

    @Override
    public void periodic() {
       queue.run();
    }
    
}
