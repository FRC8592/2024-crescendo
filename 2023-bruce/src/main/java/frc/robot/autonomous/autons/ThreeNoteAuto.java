package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;

import frc.robot.commands.FollowerCommand;

public class ThreeNoteAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(config, 
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), 
            AutonomousPositions.WING_NOTE_3.translate(0, 0.5, Rotation2d.fromDegrees(-45))).addRotation(Rotation2d.fromDegrees(45), 0.5);

    private SwerveTrajectory pathTwo = AutonomousPositions.generate(config.setReversed(false), 
            AutonomousPositions.WING_NOTE_3.translate(0, 0.5, Rotation2d.fromDegrees(180)), 
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(0))).addRotation(Rotation2d.fromDegrees(0));

    private SwerveTrajectory pathThree = AutonomousPositions.generate(config.setReversed(false), 
            AutonomousPositions.WING_NOTE_2.rotate(Rotation2d.fromDegrees(180)), 
            AutonomousPositions.WING_NOTE_1.rotate(Rotation2d.fromDegrees(0))).addRotation(Rotation2d.fromDegrees(0));

    private SwerveTrajectory pathFour = AutonomousPositions.generate(config.setReversed(false), 
            AutonomousPositions.WING_NOTE_1.getPose(), 
            AutonomousPositions.MID_NOTE_1.translate(0, 0.3));

    private SwerveTrajectory pathFive = AutonomousPositions.generate(config.setReversed(true), 
            AutonomousPositions.MID_NOTE_1.translate(0, 0.3),
            AutonomousPositions.WING_NOTE_1.translate(3.0, -0.2));

    private SwerveTrajectory pathSix = AutonomousPositions.generate(config.setReversed(false),
            AutonomousPositions.WING_NOTE_1.translate(3.0, -0.2),
            AutonomousPositions.MID_NOTE_2.getPose());

    private SwerveTrajectory pathSeven = AutonomousPositions.generate(config.setReversed(true),
            AutonomousPositions.MID_NOTE_2.getPose(),
            AutonomousPositions.WING_NOTE_1.translate(3.0,0.2));


    @Override
    public void initialize() {
        queue = new CommandQueue(
            new DelayCommand(0.75),
            new FollowerCommand(drive, pathOne, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.75),
            new FollowerCommand(drive,pathTwo, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.75),
            new FollowerCommand(drive, pathThree, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.75),
            new FollowerCommand(drive, pathFour, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.75),
            new FollowerCommand(drive, pathFive, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.75),
            new FollowerCommand(drive, pathSix, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.75),
            new FollowerCommand(drive, pathSeven, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose())

        );
        // TODO Auto-generated method stub
        
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        queue.run();
        
    }
    
}
