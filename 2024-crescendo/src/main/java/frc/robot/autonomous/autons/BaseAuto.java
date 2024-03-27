package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.*;
import frc.robot.commands.CommandQueue;

public abstract class BaseAuto {
    protected Swerve drive;
    protected Elevator elevator;
    protected Intake intake;
    protected LimelightTargeting targeting;
    protected Shooter shooter;
    protected PoseVision poseVision;
    protected MainSubsystemsManager subsystemsManager;
    protected CommandQueue queue;

    private TrajectoryConfig fastConfig = new TrajectoryConfig(1, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    /**
     * Add all running subsystems for use for all autonomous routines
     * @param pDrive {@code Swerve} object
     */
    public void addModules(Swerve pDrive, Elevator pLift, Intake pIntake, Shooter pShooter, LimelightTargeting pTargeting, PoseVision pPoseVision, MainSubsystemsManager pSubsystemsManager) {
        drive = pDrive;
        elevator = pLift;
        intake = pIntake;
        targeting = pTargeting;
        shooter = pShooter;
        poseVision = pPoseVision;
        subsystemsManager = pSubsystemsManager;
    }

    /**
     * Sets the robot's simulation start {@code Pose2d}
     */
    public void setInitialSimulationPose() {
        Robot.FIELD.setRobotPose(queue.getStartPose());
    }

    /**
     * @return starting {@code Pose2d} based on activated trajectories
     */
    public Pose2d getStartPose() {
        return queue.getStartPose();
    }

    public void addDelay(double seconds) {
        queue.addDelay(seconds);
    }

    public abstract void initialize();
    public abstract void periodic();
}