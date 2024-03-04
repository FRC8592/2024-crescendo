package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.*;
import frc.robot.Constants.*;
import frc.robot.autonomous.*;
import frc.robot.commands.*;

public class DONTSELECT_RotateTestAuto extends BaseAuto{
    private TrajectoryConfig config = new TrajectoryConfig(4,4);

    private SwerveTrajectory midNote1 = AutonomousPositions.generate(config.setEndVelocity(4),
            AutonomousPositions.SUBWOOFER_DOWN.getPose(),
            AutonomousPositions.MID_NOTE_5.translate(-5,0));

    public DONTSELECT_RotateTestAuto(){
        // SmartDashboard.p%utNumber("RotateAutoTestAngle", 0);
    }
    public void initialize() {
        queue = new CommandQueue(
                new GyroSetCommand(drive, 0),
                // new ShootCommand(shooter, elevator, 3000, 6),
                new FollowerCommand(drive, midNote1),
                new RotateCommand(drive, Rotation2d.fromDegrees(SmartDashboard.getNumber("RotateAutoTestAngle", 0))));
    }
    public void periodic() {
        queue.run();
    }
}
