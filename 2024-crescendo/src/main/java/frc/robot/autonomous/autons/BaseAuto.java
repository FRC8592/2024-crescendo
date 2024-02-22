package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.*;
import frc.robot.LimelightTargeting;
import frc.robot.commands.CommandQueue;

public abstract class BaseAuto {
    protected Swerve drive;
    protected Elevator elevator;
    protected Intake intake;
    protected LimelightTargeting targeting;
    protected Shooter shooter;
    protected CommandQueue queue;

    protected double scoreTime = 1; // Remove once actual intake/outtake on the robot

    /**
     * Add all running subsystems for use for all autonomous routines
     * @param pDrive {@code Swerve} object
     */
    public void addModules(Swerve pDrive/*, Elevator pLift, Intake pIntake, Shooter pShooter, LimelightTargeting pTargeting*/) {
        drive = pDrive;
        // elevator = pLift;
        // intake = pIntake;
        // targeting = pTargeting;
        // shooter = pShooter;
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