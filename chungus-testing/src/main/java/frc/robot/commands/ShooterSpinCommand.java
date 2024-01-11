package frc.robot.commands;
import frc.robot.*;
public class ShooterSpinCommand extends Command{
    private Shooter shooter;
    public ShooterSpinCommand(Shooter shooter){
        this.shooter = shooter;
    }
    public void initialize(){}
    public boolean execute(){
        // return shooter.spin();
        return true;
    }
    public void shutdown(){}
}
