package frc.robot.commands;
import frc.robot.*;
public class GyroSetCommand extends Command {
    Swerve swerve;
    double yaw;
    public GyroSetCommand(Swerve pSwerve, double yaw){
        swerve = pSwerve;
        this.yaw = yaw;
    }
    public void initialize() {
        
    }

    @Override
    public boolean execute() {
        this.swerve.setGyroscopeRotation(yaw);
        return true;
    }

    @Override
    public void shutdown() {
        
    }

}
