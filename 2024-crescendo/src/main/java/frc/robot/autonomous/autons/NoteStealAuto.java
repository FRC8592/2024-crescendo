package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;


/*How to add a point after you reach the position--  
 * 
 */
//IN PROGRESS, CHANGE CONFIG TO 4 SPEED AND 2 ACC
public class NoteStealAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(4.5, 4.5);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 2);
    
    private SwerveTrajectory noteOne = AutonomousPositions.generate(config.setReversed(false), 
            AutonomousPositions.SUBWOOFER_DOWN.getPose(),
            AutonomousPositions.MID_NOTE_5.translate(0, 0.3));
    
    private SwerveTrajectory shootOne = AutonomousPositions.generate(config.setReversed(true), 
            AutonomousPositions.MID_NOTE_5.translate(0,0.3),
            AutonomousPositions.WING_NOTE_3.translate(3, -2.3));

    private SwerveTrajectory noteTwo = AutonomousPositions.generate(config.setReversed(false),
            AutonomousPositions.WING_NOTE_3.translate(3, -2.3),
            AutonomousPositions.MID_NOTE_4.getPose()
    );

    private SwerveTrajectory shootTwo = AutonomousPositions.generate(config.setReversed(true),
            AutonomousPositions.MID_NOTE_4.getPose(),
            AutonomousPositions.WING_NOTE_3.translate(2.9, -2.5) 
    
    );

    private SwerveTrajectory noteThree = AutonomousPositions.generate(config.setReversed(false),
            AutonomousPositions.WING_NOTE_3.translate(2.9, -2.5),
            AutonomousPositions.MID_NOTE_4.translate(-0.5,0),
            AutonomousPositions.MID_NOTE_3.getPose()
    );

    private SwerveTrajectory shootThree = AutonomousPositions.generate(config.setReversed(true), 
            AutonomousPositions.MID_NOTE_3.getPose()
            ,
            AutonomousPositions.WING_NOTE_2.translate(1.2, -2.1)
    );

    

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new DelayCommand(0.5),
            new FollowerCommand(drive, noteOne, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(.2),
            new FollowerCommand(drive, shootOne, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.5),
            new FollowerCommand(drive, noteTwo, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(.2),
            new FollowerCommand(drive, shootTwo, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.5),
            new FollowerCommand(drive, noteThree, AutonomousPositions.SUBWOOFER_MIDDLE_2.getPose()),
            new DelayCommand(0.2),
            new FollowerCommand(drive, shootThree)

        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
