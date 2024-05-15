package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;

public class Controls {
    public boolean resetGyro;
    public boolean autoAim;
    
    public boolean partyMode;
    public boolean intake;
    public boolean ledAmpSignal;
    public boolean rangeTableShoot;
    public boolean stow;
    public boolean amp;
    public boolean climb;
    public boolean score;
    public boolean manualExtend;
    public boolean manualRetract;
    public boolean manualPivotIncrease;
    public boolean manualPivotDecrease;

    public Controls(){
        this.resetGyro = false;
        this.intake = false;
        this.autoAim = false;
        this.partyMode = false;
        this.ledAmpSignal = false;
        this.rangeTableShoot = false;
        this.stow = false;
        this.amp = false;
        this.climb = false;
        this.score = false;
        this.manualExtend = false;
        this.manualRetract = false;
        this.manualPivotDecrease = false;
        this.manualPivotIncrease = false;
    }

    public void update(XboxController driverController, XboxController operatorController){
        this.resetGyro = driverController.getBackButton();

        this.autoAim = operatorController.getLeftBumper();
        this.score = operatorController.getRightTriggerAxis()>0.1;
        this.partyMode = operatorController.getStartButton();
        this.intake = operatorController.getLeftTriggerAxis() > 0.1;
        this.rangeTableShoot = operatorController.getRightBumper();
        this.stow = operatorController.getAButton();
        this.amp = operatorController.getXButton();
        this.climb = operatorController.getYButton();
        this.manualExtend = operatorController.getPOV() == 0;
        this.manualRetract = operatorController.getPOV() == 180;
        this.manualPivotIncrease = operatorController.getPOV() == 90;
        this.manualPivotDecrease = operatorController.getPOV() == 270;
        this.ledAmpSignal = operatorController.getBackButton();
    }
    private void log(boolean logged, String controller, String name){
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name, logged);
    }
}
