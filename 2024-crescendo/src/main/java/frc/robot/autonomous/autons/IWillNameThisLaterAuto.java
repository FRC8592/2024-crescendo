package frc.robot.autonomous.autons;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;

public class IWillNameThisLaterAuto extends BaseAuto{

    //setting speed and acceleration for paths
    private TrajectoryConfig config = new TrajectoryConfig(4, 2);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 2);

    private SwerveTrajectory noteOne = AutonomousPositions.generate(config, 
            AutonomousPositions.SUBWOOFER_UP.getPose(),
            AutonomousPositions.WING_NOTE_1.getPose());
    
    private SwerveTrajectory noteTwo = AutonomousPositions.generate(config, 
            AutonomousPositions.WING_NOTE_1.getPose(),
            AutonomousPositions.MID_NOTE_1.getPose());

    private SwerveTrajectory shootTwo = AutonomousPositions.generate(config.setReversed(true), 
            AutonomousPositions.MID_NOTE_1.getPose(),
            AutonomousPositions.WING_NOTE_1.translate(3, -0.2));

    private SwerveTrajectory noteThree = AutonomousPositions.generate(config.setReversed(false), 
            AutonomousPositions.WING_NOTE_1.translate(3, -0.2),
            AutonomousPositions.MID_NOTE_2.getPose());

    private SwerveTrajectory shootThree = AutonomousPositions.generate(config.setReversed(true), 
            AutonomousPositions.MID_NOTE_2.getPose(),
            AutonomousPositions.WING_NOTE_1.translate(3, -0.2));

    @Override
    public void initialize() {
         queue = new CommandQueue(
            new DelayCommand(0.25),
            new FollowerCommand(drive, noteOne, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.25),
            new FollowerCommand(drive, noteTwo, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.25),
            new FollowerCommand(drive, shootTwo, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.25),
            new FollowerCommand(drive, noteThree),
            new DelayCommand(0.25),
            new FollowerCommand(drive, shootThree)


         );
    }

    @Override
    public void periodic() {
      queue.run();
    }
    
}
