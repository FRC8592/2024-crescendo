package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.ShootCommand;



public class ThreeWingNoteAuto extends BaseAuto{

    //Setting speed and acceleration of each path as shown
    private TrajectoryConfig config = new TrajectoryConfig(2, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);
   
    private SwerveTrajectory noteOne_1 = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0), 
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), 
            AutonomousPositions.WING_NOTE_3.translate(0, 0.4, Rotation2d.fromDegrees(-45))).addRotation(Rotation2d.fromDegrees(-45));//-45


    private SwerveTrajectory noteTwo_1 = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(0).setEndVelocity(2), 
            AutonomousPositions.WING_NOTE_3.translate(0, 0.4, Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_2.translate(-0.5, -0.2, Rotation2d.fromDegrees(-90)));

    private SwerveTrajectory noteTwo_2 = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(2).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_2.translate(-0.5, -0.2, Rotation2d.fromDegrees(-90)),
            AutonomousPositions.WING_NOTE_2.translate(0.3, -0.1, Rotation2d.fromDegrees(0))).addRotation(Rotation2d.fromDegrees(0));


    private SwerveTrajectory noteThree_1 = AutonomousPositions.generate(config.setReversed(true).setStartVelocity(0).setEndVelocity(2), 
            AutonomousPositions.WING_NOTE_2.translate(0.1, -0.1,Rotation2d.fromDegrees(0)),
            AutonomousPositions.WING_NOTE_1.translate(-0.5, -0.5, Rotation2d.fromDegrees(-90))).addRotation(Rotation2d.fromDegrees(25));

    private SwerveTrajectory noteThree_2 = AutonomousPositions.generate(config.setReversed(true).setStartVelocity(2).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_1.translate(-0.5, -0.5, Rotation2d.fromDegrees(-90)),
            AutonomousPositions.WING_NOTE_1.translate(0.2, -0.3,Rotation2d.fromDegrees(0))).addRotation(Rotation2d.fromDegrees(25));


    @Override
    public void initialize() {
       queue = new CommandQueue(
        new ShootCommand(shooter, elevator, 1.4), //shooting preloaded note
        new JointCommand(//going to first note and intaking at the same time
                new FollowerCommand(drive, noteOne_1.addVision(targeting, -15)),
                new IntakeCommand(intake, shooter).setBottomSensorTripTimeout(2).setTimeout(4)
        ),
        new DelayCommand(0.2),
        new AutoShootCommand(drive, poseVision, elevator, shooter), //shooting first note at wing position
        new FollowerCommand(drive, noteTwo_1),
        new JointCommand(//going to and intaking second note
                new FollowerCommand(drive, noteTwo_2.addVision(targeting, -15)),
                new IntakeCommand(intake, shooter).setBottomSensorTripTimeout(2).setTimeout(4)
        ),
        new DelayCommand(0.2),
        new AutoShootCommand(drive, poseVision, elevator, shooter), //shooting second note at wing spot
        new FollowerCommand(drive, noteThree_1),
        new JointCommand(//going to and intaking thrid note
                 new FollowerCommand(drive, noteThree_2.addVision(targeting, -15)),
                 new IntakeCommand(intake, shooter).setBottomSensorTripTimeout(2).setTimeout(4)
        ),
        new DelayCommand(0.2),
        new AutoShootCommand(drive, poseVision, elevator, shooter) //shooting third note 
       );
    }

    
    @Override
    public void periodic() {
        queue.run();
    }
    
}
