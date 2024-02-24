package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Elevator;
import frc.robot.Shooter;
import frc.robot.Constants.*;

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
        timeoutTimer.start();
    }

    @Override
    public boolean execute() {
        elevator.setPivotAngleCustom(elevatorPivotAngle);
        shooter.setShootVelocity(shootVelocity, shootVelocity);

        if(shooter.isReady() && Math.abs(elevator.getPivotAngle() - elevatorPivotAngle) <= 0.5){
            timer.start();
            if(timer.get()>0.05){
                shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED);
            }
            else{
                shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_VELOCITY);
            }
        }

        return timer.get() > 1.5 || (timeoutSeconds != -1 && timeoutTimer.get() >= timeoutSeconds);
    }

    @Override
    public void shutdown() {
      shooter.stop();
      shooter.stopFeeders();
      elevator.stow();
    }
    
}
