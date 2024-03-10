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

public class FollowerCommand extends Command {
    private Swerve drive;
    private SwerveTrajectory trajectory;
    private Timer timer;
    private LimelightTargeting targeting;
    private PIDController visionPID;
    // private Rotation2d endRotation;
    private Pose2d targetPose;
    // private SmoothingFilter omegaSmoothing;
    private Rotation2d simulatedRotation = new Rotation2d();

    public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj) {
        drive = pDrive;
        trajectory = pTraj;
        timer = new Timer();
    }

    public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj, String tag) {
        drive = pDrive;
        trajectory = pTraj;
        timer = new Timer();
        setTag(tag);
        targetPose = null;
    }

    public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj, Rotation2d pRot) {
        drive = pDrive;
        trajectory = pTraj.addRotation(pRot);
        timer = new Timer();
        targetPose = null;

        // endRotation = pRot;
    }

    public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj, Rotation2d pRot, String tag) {
        drive = pDrive;
        trajectory = pTraj.addRotation(pRot);
        timer = new Timer();
        setTag(tag);
        targetPose = null;

        // endRotation = pRot;
    }

    public FollowerCommand(Swerve drive, LimelightTargeting targeting, SwerveTrajectory pTraj) {
        this.drive = drive;
        this.targeting = targeting;
        trajectory = pTraj;
        visionPID = new PIDController(NOTELOCK.TURN_kP, NOTELOCK.TURN_kI, NOTELOCK.TURN_kD);
        timer = new Timer();
        targetPose = null;
    }

    public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj, Pose2d pTargetPose) {
        this.drive = pDrive;
        this.trajectory = pTraj;
        this.targetPose = pTargetPose;
        this.timer = new Timer();
    }

    public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj, Pose2d pTargetPose, LimelightTargeting targeting) {
        this.drive = pDrive;
        this.trajectory = pTraj;
        this.targetPose = pTargetPose;
        this.targeting = targeting;
        this.visionPID = new PIDController(NOTELOCK.TURN_kP, NOTELOCK.TURN_kI, NOTELOCK.TURN_kD);
        // this.omegaSmoothing = new SmoothingFilter(1, 1, 1); // x, y, omega
        this.timer = new Timer();
    }

    public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj, Pose2d pTargetPose, LimelightTargeting targeting, PIDController internalPID) {
        this.drive = pDrive;
        this.trajectory = pTraj;
        this.targetPose = pTargetPose;

        if(DriverStation.getAlliance().get() == Alliance.Red) {
            // flip if its red
            this.targetPose = new Pose2d(16 - pTargetPose.getX(), pTargetPose.getY(), pTargetPose.getRotation());
        }

        this.targeting = targeting;
        this.visionPID = internalPID;
        // this.omegaSmoothing = new SmoothingFilter(1, 1, 1); // x, y, omega
        this.timer = new Timer();
    }

    // public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj, Rotation2d
    // pRot, boolean lockWheels) {
    // drive = pDrive;
    // trajectory = pTraj.addRotation(pRot);
    // timer = new Timer();
    // setTag(tag);

    // endRotation = pRot;
    // }

    // public FollowerCommand(Swerve pDrive, SwerveTrajectory pTraj, Rotation2d
    // pRot, boolean lockWheels, String tag) {
    // drive = pDrive;
    // trajectory = pTraj.addRotation(pRot);
    // timer = new Timer();
    // setTag(tag);

    // endRotation = pRot;
    // }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        ChassisSpeeds speeds = trajectory.sample(timer.get(), drive.getCurrentPos());
        double time = timer.get();
        boolean finishedAiming = false;

        if (!Robot.isReal()) {
            simulateRobotPose(trajectory.trajectory().sample(time).poseMeters, speeds);
        }
        ChassisSpeeds newSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);
        Pose2d idealPose = new Pose2d();
        if (targeting != null) { // if we have vision
            targeting.updateVision();
            double omegaVision;
            if (targeting.isTargetValid()) { // target in view
                omegaVision = targeting.turnRobot(0, visionPID, "tx", 2, 0.0);
                if (omegaVision == 0) {
                    finishedAiming = true;
                }
                // System.out.println("AprilTag Found, omega = " + omegaVision);
                // SmartDashboard.putNumber("AprilTag Omega", omegaVision);
                Logger.recordOutput("CustomLogs/Autonomous/VisionRotationTarget", omegaVision);
                Logger.recordOutput("CustomLogs/Autonomous/VisionOffset", targeting.tx.getDouble(0.0));
                Logger.recordOutput("CustomLogs/Autonomous/GyroscopeRotation",
                        drive.getCurrentPos().getRotation().getDegrees());
                idealPose = new Pose2d(trajectory.trajectory().sample(time).poseMeters.getTranslation(), // Same translation as the target from the trajectory
                        drive.getCurrentPos().getRotation().plus(Rotation2d.fromDegrees(-targeting.tx.getDouble(0.0)))); // Current rotation plus the offset from the april tag target
                Logger.recordOutput("CustomLogs/Autonomous/SeesAprilTag", true);
                Logger.recordOutput("CustomLogs/Autonomous/TargetX", targeting.tx.getDouble(0.0));
                Logger.recordOutput("CustomLogs/Autonomous/TargetY", targeting.ty.getDouble(0.0));
            } else {
                Logger.recordOutput("CustomLogs/Autonomous/SeesAprilTag", false);
                if (targetPose != null) { // have a target pose
                    Pose2d currentPose;
                    if (Robot.isReal()) {
                        currentPose = drive.getCurrentPos();
                    } else {
                        currentPose = trajectory.trajectory().sample(time).poseMeters;
                    }
                    double angleRadians = Math.atan2(targetPose.getY() - currentPose.getY(),
                            targetPose.getX() - currentPose.getX());

                    if (!Robot.isReal()) {
                        angleRadians += Math.PI;
                    }

                    Rotation2d targetRotation = new Rotation2d(angleRadians);
                    trajectory.setRotation(targetRotation);
                    System.out.println(targetRotation.getDegrees());
                    idealPose = new Pose2d(trajectory.trajectory().sample(time).poseMeters.getTranslation(), // Same
                                                                                                             // translation
                                                                                                             // as the
                                                                                                             // target
                                                                                                             // from the
                                                                                                             // trajectory
                            targetRotation); // Rotation

                    simulatedRotation = targetRotation;
                }
                omegaVision = speeds.omegaRadiansPerSecond; // if we don't have vision, just
                // use the trajectory's omega
                // omegaVision = 0; // don't turn if you don't see apriltag
            }
            Logger.recordOutput("CustomLogs/Autonomous/TargetPose", idealPose);
            // if
            // (Math.abs(trajectory.trajectory().sample(trajectory.trajectory().getTotalTimeSeconds()).poseMeters.getX()
            // - drive.getCurrentPos().getX()) <= 0.2) {
            // omegaVision = 0;
            // }

            // SmartDashboard.putNumber("VY Speed From Vision", vyVision);

            newSpeeds = new ChassisSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    omegaVision);
            // newSpeeds = omegaSmoothing.smooth(newSpeeds);
        }
        Logger.recordOutput("CustomLogs/Autonomous/ActualPose", drive.getCurrentPos()); // Log where the robot actually
                                                                                        // is. This is still
                                                                               // dead-reckoning.
        // Robot.FIELD.getObject("RobotTrajectory").setTrajectory(trajectory.trajectory());
        drive.drive(newSpeeds);

        return trajectory.isFinished(time) || trajectory.isVision;
    }

    @Override
    public Pose2d getStartPose() {
        return trajectory.getInitialPose();
    }

    private void simulateRobotPose(Pose2d pose, ChassisSpeeds desiredSpeeds) {
        Trajectory traj = trajectory.trajectory();
        Rotation2d rotation = trajectory.getRotation();
        if (targetPose != null) {
            rotation = simulatedRotation;
        }
        Robot.FIELD.setRobotPose(new Pose2d(pose.getTranslation(), rotation));

        // SmartDashboard.putNumber("Field Relative X Velocity",
        // desiredSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("Field Relative Y Velocity",
        // desiredSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber("Field Relative Omega",
        // desiredSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void shutdown() {
        // if (lockWheels) {
        // drive.setWheelLock();
        // } else {
        drive.drive(new ChassisSpeeds());
        // }
        // drive.drive(trajectory.sample(trajectory.trajectory().getTotalTimeSeconds() -
        // 0.02, drive.getCurrentPos()));
    }
}