package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Elevator;
import frc.robot.Shooter;

public class ShootCommand extends Command{
    private Shooter shooter;
    private Elevator elevator;
    private int shootVelocity;
    private double elevatorPivotAngle;
    private Timer timer = new Timer();

    public ShootCommand(Shooter shooter, Elevator elevator, int shootVelocity, double elevatorPivotAngle){
        this.shooter = shooter;
        this.elevator = elevator;
        this.shootVelocity = shootVelocity;
        this.elevatorPivotAngle = elevatorPivotAngle;
    }
    
    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public boolean execute() {
        elevator.setPivotAngleCustom(elevatorPivotAngle);
        shooter.setShootVelocity(shootVelocity, shootVelocity);

        if(shooter.isReady()){
            shooter.setFeederVelocity(2000);
            timer.start();
        }
        
        return timer.get() > 1.5;
    }

    @Override
    public void shutdown() {
      shooter.stop();
      shooter.stopFeeders();
      elevator.stow();
    }
    
}
