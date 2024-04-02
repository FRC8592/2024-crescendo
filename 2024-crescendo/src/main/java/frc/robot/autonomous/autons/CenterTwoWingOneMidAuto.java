package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.JointCommand;

public class CenterTwoWingOneMidAuto extends BaseAuto{

    private TrajectoryConfig config = new TrajectoryConfig(1, 3);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 3);
   
    private SwerveTrajectory noteOne = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0), 
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), 
            AutonomousPositions.WING_NOTE_3.translate(-0.1, 0.6, Rotation2d.fromDegrees(-45))).addRotation(Rotation2d.fromDegrees(-45));//-45


    private SwerveTrajectory noteTwo = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(0).setEndVelocity(0), 
            AutonomousPositions.WING_NOTE_3.translate(-0.1, 0.6, Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_2.translate(0.3,-0.1));

    private SwerveTrajectory noteThree = AutonomousPositions.generate(config.setReversed(false),    
            AutonomousPositions.WING_NOTE_2.translate(0.3,-0.1),
            AutonomousPositions.MID_NOTE_3.translate(-1,0));

    @Override
    public void initialize() {
      queue = new CommandQueue(
        new FollowerCommand(drive, noteOne),
        new FollowerCommand(drive, noteTwo)

      );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
