package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.Constants.*;
import frc.robot.MainSubsystemsManager.MechanismState;

public class MovingShotCommand extends Command{
    private MainSubsystemsManager subsystemsManager;
    private double expectedRange;
    private Timer timer = new Timer();
    private Controls controls = new Controls();
    private FollowerCommand followerCommand;
    private Pose2d targetPose;
    private Swerve swerve;

    public MovingShotCommand(MainSubsystemsManager subsystemsManager, double expectedRange, FollowerCommand followerCommand, Pose2d targetShot, Swerve swerve){
        this.subsystemsManager = subsystemsManager;
        this.expectedRange = expectedRange;
        this.followerCommand = followerCommand;
        this.targetPose = targetShot;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        controls = new Controls();
        timer.reset();
        timeoutTimer.start();
        subsystemsManager.staticPrime(RangeTable.get(expectedRange));
        controls.shootFromPodium = true; // Any static shot will work
        followerCommand.initialize();
    }

    @Override
    public boolean execute() {
        if (!Robot.isReal()) {
            return true;
        }

        Logger.recordOutput("CurrentCommand", "ShootCommand");
        controls.score = true;
        subsystemsManager.staticPrime(RangeTable.get(expectedRange));
        subsystemsManager.updateMechanismStateMachine(controls, 0, true);
        if(isWithinDistance(targetPose, swerve.getCurrentPos(), 0.5)){
            controls.score = true;
        }
        else{
            controls.score = false;
        }
        return subsystemsManager.mechanismState == MechanismState.STOWING;
    }

    @Override
    public void shutdown() {
        controls.score = false;
        subsystemsManager.updateMechanismStateMachine(controls, 0, true);
    }
    private boolean isWithinDistance(Pose2d one, Pose2d two, double distanceMeters){
        return Math.sqrt(Math.pow(two.getX()-one.getX(), 2)+Math.pow(two.getY()-one.getY(), 2)) <= Math.abs(distanceMeters);
    }
}
