package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public abstract class Command {
    protected String tag = "DEFAULT COMMAND";
    protected Timer timeoutTimer = new Timer();
    protected double timeoutSeconds = -1;

    /**
     * Setup the command; called directly before the command is run
     */
    public abstract void initialize();

    /**
     * Ran periodically for the command
     * 
     * @return whether or not the command has finished
     */
    public abstract boolean execute();

    /**
     * Ran after the command has finished; typically for turning off or resetting a
     * mechanism
     */
    public abstract void shutdown();

    /**
     * Sets a traceable tag for the given command; useful for cherry-picking a
     * certain command out of the queue
     */
    public Command setTag(String tag) {
        this.tag = tag;
        return this;
    }

    /**
     * Set the timeout for the command
     * @param timeout in seconds
     * @return this
     */
    public Command setTimeout(double timeout) {
        this.timeoutSeconds = timeout;
        return this;
    }

    /**
     * @return {@code String} representing the given tag for the command
     */
    public String tag() {
        return tag;
    }

    /**
     * Any command that changes the position or orientation of the robot should
     * {@code @Override} this method
     * 
     * @return {@code Pose2d} representing the starting position of the robot
     */
    public Pose2d getStartPose() {
        return new Pose2d();
    }
}