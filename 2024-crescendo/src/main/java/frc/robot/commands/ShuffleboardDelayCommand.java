package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardDelayCommand extends Command {
    private Timer timer = new Timer();
    private double seconds;

    public ShuffleboardDelayCommand() {
        // SmartDashboard.putNumber("Auto Delay", 0);
    }

    public void setDelay() {
        this.seconds = SmartDashboard.getNumber("Auto Delay", 0);
    }

    @Override
    public void initialize() {
        setDelay();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        timer.start();
        return timer.get() >= this.seconds;
    }

    @Override
    public void shutdown() {
        timer.reset();
    }

}
