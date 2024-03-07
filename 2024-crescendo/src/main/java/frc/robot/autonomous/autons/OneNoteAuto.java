package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.*;

public class OneNoteAuto extends BaseAuto {


    @Override
    public void initialize() {
        queue = new CommandQueue(
                new ShootCommand(shooter, elevator, 3000, 5)    
               
       );
    }

    @Override
    public void periodic() {
        queue.run();
    }

}
