package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;



public class ThreeWingNoteAuto extends BaseAuto{

    //Setting speed and acceleration of each path as shown
    private TrajectoryConfig config = new TrajectoryConfig(4.5, 4.5);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 2);
   
    private SwerveTrajectory noteOne = AutonomousPositions.generate(config, 
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), 
            AutonomousPositions.WING_NOTE_3.translate(0, 0.5, Rotation2d.fromDegrees(-45)));

    private SwerveTrajectory noteTwo = AutonomousPositions.generate(config.setReversed(false).setEndVelocity(0.5), 
            AutonomousPositions.WING_NOTE_3.translate(0, 0.5, Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(0)));

    private SwerveTrajectory noteThree = AutonomousPositions.generate(config.setReversed(true).setEndVelocity(1.5), 
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(0)),
            AutonomousPositions.WING_NOTE_1.rotate(Rotation2d.fromDegrees(215)),
            AutonomousPositions.WING_NOTE_1.translate(2, 0, Rotation2d.fromDegrees(180))
            );

    private SwerveTrajectory midLine = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(1.5), 
            AutonomousPositions.WING_NOTE_1.translate(2, 0), 
            AutonomousPositions.MID_NOTE_1.translate(-0.9, 0));


    @Override
    public void initialize() {
       queue = new CommandQueue(
        new DelayCommand(0.25),
        new FollowerCommand(drive, noteOne),
        new DelayCommand(0.25),
        new FollowerCommand(drive, noteTwo),
        new DelayCommand(0.25),
        new FollowerCommand(drive, noteThree),
        new FollowerCommand(drive, midLine)
       );
    }

    
    @Override
    public void periodic() {
        queue.run();
    }
    
}
