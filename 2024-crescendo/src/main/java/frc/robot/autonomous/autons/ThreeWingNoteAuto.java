package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;



public class ThreeWingNoteAuto extends BaseAuto{

    //   
    // Setting speed and acceleration of each path as shown
    //
    private TrajectoryConfig config = new TrajectoryConfig(2, 3);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 4);
   
    private SwerveTrajectory noteOne_1 = AutonomousPositions.generate(config.setStartVelocity(0).setEndVelocity(0), 
            AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), 
            AutonomousPositions.WING_NOTE_3.translate(0.2, 0.5, Rotation2d.fromDegrees(-45))).addRotation(Rotation2d.fromDegrees(-45));
        
    private SwerveTrajectory noteTwo_1 = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(0).setEndVelocity(0), 
            AutonomousPositions.WING_NOTE_3.translate(0.2, 0.5, Rotation2d.fromDegrees(180)),
            AutonomousPositions.WING_NOTE_2.translate(-0.7, -0.1, Rotation2d.fromDegrees(-90)));

    private SwerveTrajectory noteTwo_2 = AutonomousPositions.generate(config.setReversed(false).setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_2.translate(-0.7, -0.1),
            AutonomousPositions.WING_NOTE_2.translate(0.3, 0.0, Rotation2d.fromDegrees(0))).addRotation(Rotation2d.fromDegrees(0));

    private SwerveTrajectory noteThree_1 = AutonomousPositions.generate(config.setReversed(true).setStartVelocity(0).setEndVelocity(0), 
            AutonomousPositions.WING_NOTE_2.translate(0.3, 0.0,Rotation2d.fromDegrees(0)), // -0.3
            AutonomousPositions.WING_NOTE_1.translate(-0.5, -0.3, Rotation2d.fromDegrees(-90))).addRotation(Rotation2d.fromDegrees(20));

    private SwerveTrajectory noteThree_2 = AutonomousPositions.generate(config.setReversed(true).setStartVelocity(0).setEndVelocity(0),
            AutonomousPositions.WING_NOTE_1.translate(-0.5, -0.3),
            AutonomousPositions.WING_NOTE_1.translate(0.1, 0.3,Rotation2d.fromDegrees(0))).addRotation(Rotation2d.fromDegrees(15));


    @Override
    public void initialize() {
       queue = new CommandQueue(
        // Shoot the preloaded note while at the base of the subwoofer
        new ShootCommand(subsystemsManager, 1.4), //shooting preloaded note

        // Move straight towards the first wing note while intaking
        // This operation is highly reliable, so vision is disabled
        new JointCommand(
                new FollowerCommand(drive, noteOne_1.addVision(targeting, -18)),
                new IntakeCommand(subsystemsManager).setBottomSensorTripTimeout(2).setTimeout(4)
        ),

        // Drive away from the first wing note in an arc to the second note.  We'll shoot at the end of this movement
        new JointCommand(
            new PrimeCommand(subsystemsManager, 1.4), // Spin up the shooter wheels in advance to reduce time-to-shoot
            new FollowerCommand(drive, noteTwo_1)
        ),

        // Shoot the first wing note using AprilTag aiming
        new AutoShootCommand(drive, poseVision, subsystemsManager),

        // Move directly back towards the second wing note while intaking
        new JointCommand(
                new FollowerCommand(drive, noteTwo_2.addVision(targeting, -10)),
                new IntakeCommand(subsystemsManager).setBottomSensorTripTimeout(2).setTimeout(4)
        ),

        // Move away from the second wing note in an arc to the third wing note. we'll shoot at the end of this movement.
        new JointCommand(
            new PrimeCommand(subsystemsManager, 2),  // Spin up shooter wheels to reduce shooting time
            new FollowerCommand(drive, noteThree_1)
        ),

        // Shoot the second wing note using AprilTag aiming
        new AutoShootCommand(drive, poseVision, subsystemsManager), //shooting second note at wing spot

        // Move towards the third wing note while collecting
        // Positioning on this note seems to often give us trouble, so the vision is enabled early
        new JointCommand(//going to and intaking thrid note
                 new FollowerCommand(drive, noteThree_2.addVision(targeting, 0)),
                 new IntakeCommand(subsystemsManager).setBottomSensorTripTimeout(2).setTimeout(4)
        ),

        // We don't know why, but we frequently have trouble acquiring the speaker AprilTag from this position.
        // For this reason, we add a short delay so the robot doesn't move to help in tag acquisition
        //
        // TODO: We could try priming the shooter here to save time.  However, this could cause unwanted camera shake
        new DelayCommand(0.2),

        // Shoot the third wing note
        new AutoShootCommand(drive, poseVision, subsystemsManager)
       );
    }

    
    @Override
    public void periodic() {
        queue.run();
    }
    
}
