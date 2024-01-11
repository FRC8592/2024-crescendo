package frc.robot.commands;
import frc.robot.*;
public class ShootCommand extends Command{
    private Shooter shooter;
    private Intake intake;

    public ShootCommand(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
    }
    public void initialize(){}
    public boolean execute(){
        shooter.shoot(intake);
        return true; //TODO: Until we get a real command.
    }
    public void shutdown(){};
}
