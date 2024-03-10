package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        Logger.recordOutput("CurrentCommand", "ShootCommand");

        elevator.setPivotAngleCustom(elevatorPivotAngle);
        shooter.setShootVelocity(shootVelocity, shootVelocity);
        SmartDashboard.putBoolean("Shooter is ready", shooter.isReady());
        if(shooter.isReady() && elevator.isTargetAngle()){
            timer.start();
            if(timer.get()>0.05){
                shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED);
            }
            else{
                shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
            }
        }

        return timer.get() > 0.75 || (timeoutSeconds != -1 && timeoutTimer.get() >= timeoutSeconds);
    }

    @Override
    public void shutdown() {
      shooter.stop();
      shooter.stopFeeders();
      elevator.stow();

    }
    
}
