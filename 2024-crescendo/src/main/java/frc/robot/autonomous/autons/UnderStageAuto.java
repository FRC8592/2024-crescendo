package frc.robot.autonomous.autons;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;

public class UnderStageAuto extends BaseAuto {

    //Setting speed and acceleration for the paths as shown
    private TrajectoryConfig config = new TrajectoryConfig(4, 2);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 2);

    //Paths that the robot will take in Auto 
    private SwerveTrajectory noteOne = AutonomousPositions.generate(config, 
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(),
            AutonomousPositions.WING_NOTE_2.translate(0.5,0)
    );

    private SwerveTrajectory noteTwo = AutonomousPositions.generate(config, 
            AutonomousPositions.WING_NOTE_2.translate(0.5, 0),
            AutonomousPositions.MID_NOTE_3.translate(-2, 0),
            AutonomousPositions.MID_NOTE_3.translate(0, -0.1)
    );

    private SwerveTrajectory shootTwo = AutonomousPositions.generate(config.setReversed(true), 
            AutonomousPositions.MID_NOTE_3.translate(0, -0.1),
            AutonomousPositions.MID_NOTE_3.translate(-2.2, -0.4, Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_2.translate(1.2, -2.1)
    );
    
    @Override
    public void initialize() {
       queue = new CommandQueue(
            new DelayCommand(0.5),
            new FollowerCommand(drive, noteOne),
            new DelayCommand(0.25),
            new FollowerCommand(drive, noteTwo),
            new FollowerCommand(drive, shootTwo)

       );
    }

    @Override
    public void periodic() {
       queue.run();
    }
    
}
