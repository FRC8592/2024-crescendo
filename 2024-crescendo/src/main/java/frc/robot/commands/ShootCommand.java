package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Elevator;
import frc.robot.RangeTable;
import frc.robot.Shooter;
import frc.robot.Constants.*;

public class ShootCommand extends Command{
    private Shooter shooter;
    private Elevator elevator;
    private double expectedRange;
    private double elevatorPivotAngle;
    private Timer timer = new Timer();

    public ShootCommand(Shooter shooter, Elevator elevator, double expectedRange){
        this.shooter = shooter;
        this.elevator = elevator;
        this.expectedRange = expectedRange;
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

        RangeTable.RangeEntry entry = RangeTable.get(expectedRange);
        elevator.setPivotAngleCustom(entry.pivotAngle);
        shooter.setShootVelocity(entry.leftFlywheelSpeed, entry.rightFlywheelSpeed);
        SmartDashboard.putBoolean("Shooter is ready", shooter.readyToShoot());
        if(shooter.readyToShoot() && elevator.isTargetAngle()){
            timer.start();
            if(timer.get()>0.1){
                shooter.setFeederPower(SHOOTER.SHOOTING_FEEDER_POWER);
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
