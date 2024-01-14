package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.*;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.*;
public abstract class BaseAuto {
    protected Swerve swerve;
    protected Intake intake;
    protected Vision vision;
    protected Shooter shooter;
    protected BunnyDropper dropper;
    protected CommandQueue queue;

    protected double scoreTime = 1; // Remove once actual intake/outtake on the robot

    /**
     * Add all running subsystems for use for all autonomous routines
     * @param pDrive {@code drivetrain} object
     */
    public void addModules(Swerve pDrive, Intake pIntake, Vision pVision, Shooter pShooter, BunnyDropper pDropper) {
        swerve = pDrive;
        intake = pIntake;
        vision = pVision;
        shooter = pShooter;
        dropper = pDropper;
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