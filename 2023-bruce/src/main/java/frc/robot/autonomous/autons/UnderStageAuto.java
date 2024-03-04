package frc.robot.autonomous.autons;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;

public class UnderStageAuto extends BaseAuto{


    private TrajectoryConfig config = new TrajectoryConfig(1, 2);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 1);

    private SwerveTrajectory pathOne = AutonomousPositions.generate(slowConfig,
        AutonomousPositions.SUBWOOFER_MIDDLE.getPose(),
        AutonomousPositions.WING_NOTE_2.getPose());

    private SwerveTrajectory pathTwo = AutonomousPositions.generate(slowConfig, 
        AutonomousPositions.WING_NOTE_2.getPose(),
        AutonomousPositions.WING_NOTE_2.translate(2, -0.6),
        AutonomousPositions.WING_NOTE_2.translate(4, -0.55),
        AutonomousPositions.MID_NOTE_2.translate(0, -0.45));

    private SwerveTrajectory shootOne = AutonomousPositions.generate(slowConfig.setReversed(true), 
        AutonomousPositions.MID_NOTE_2.translate(0, -0.45),
        AutonomousPositions.WING_NOTE_1.translate(3, -0.2));
    
    private SwerveTrajectory pathThree = AutonomousPositions.generate(slowConfig.setReversed(false), 
        AutonomousPositions.WING_NOTE_1.translate(3, -0.2),
        AutonomousPositions.MID_NOTE_1.translate(0, -0.3));

    private SwerveTrajectory shootTwo = AutonomousPositions.generate(slowConfig.setReversed(true), 
        AutonomousPositions.MID_NOTE_1.translate(0, -0.3),
        AutonomousPositions.WING_NOTE_1.translate(3, -0.2));
    
    @Override
    public void initialize() {
        queue = new CommandQueue(
            new DelayCommand(0.2),
            new FollowerCommand(drive, pathOne),
            new FollowerCommand(drive, pathTwo),
            new FollowerCommand(drive, shootOne),
            new FollowerCommand(drive, pathThree),
            new FollowerCommand(drive, shootTwo)

        );  
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
