package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Swerve;
import frc.robot.Constants.NOTELOCK;
import frc.robot.Robot;
import frc.robot.SmoothingFilter;
import frc.robot.LimelightTargeting;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.SmoothingFilter;
import frc.robot.commands.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;


public class TargetCommand extends Command {
    private Swerve drive;
    private Timer timer;

    private Pose2d startPos;
    private Pose2d goalPos;

    private TrajectoryConfig config;
    private SwerveTrajectory generatedTraj;
    private FollowerCommand followcmd;

    LimelightTargeting targeting;
    PIDController visionPID;

    public TargetCommand(Swerve pDrive, Pose2d goal, TrajectoryConfig conf) {
        drive = pDrive;
        timer = new Timer();
        goalPos = goal;
        config = conf;
    }

    public TargetCommand(Swerve pDrive, Pose2d goal, TrajectoryConfig conf, LimelightTargeting limelight, PIDController internalPID) {
        drive = pDrive;
        timer = new Timer();
        goalPos = goal;
        config = conf;

        targeting = limelight;
        visionPID = internalPID;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        startPos = drive.getCurrentPos();

        generatedTraj = AutonomousPositions.generate(config, startPos, goalPos);

        if (targeting == null && visionPID == null) {
            followcmd = new FollowerCommand(drive, generatedTraj, goalPos);   
        }
        else {
            followcmd = new FollowerCommand(drive, generatedTraj, goalPos, targeting, visionPID);
        }
    }

    @Override
    public boolean execute() {
        if (!Robot.isReal()) {
            return true;
        }

        return followcmd.execute();
    }

    @Override
    public void shutdown() {
        followcmd.shutdown();
    }
}