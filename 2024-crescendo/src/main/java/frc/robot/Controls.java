package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;

public class Controls {
    public boolean slowMode;
    public boolean resetGyro;
    public boolean autoCollect;
    public boolean robotOriented;
    public boolean passThrough;
    public boolean partyMode;
    public boolean passAim;
    public boolean forceShoot;

    public boolean ledAmpSignal;
    public boolean shootFromPodium;
    public boolean rangeTableShoot;
    public boolean outake;
    public boolean intake;
    public boolean stow;
    public boolean amp;
    public boolean climb;
    public boolean score;
    public boolean manualExtend;
    public boolean manualRetract;
    public boolean kiddyPoolShot;
    public boolean trapPrime;
    public float snapToDirection;

    public Controls(){
        this.slowMode = false;
        this.resetGyro = false;
        this.autoCollect = false;
        this.robotOriented = false;
        this.passThrough = false;
        this.partyMode = false;
        this.passAim = false;
        this.forceShoot = false;

        this.ledAmpSignal = false;
        this.shootFromPodium = false;
        this.rangeTableShoot = false;
        this.outake = false;
        this.intake = false;
        this.stow = false;
        this.amp = false;
        this.climb = false;
        this.score = false;
        this.manualExtend = false;
        this.manualRetract = false;
        this.kiddyPoolShot = false;
        this.trapPrime = false;
        this.snapToDirection = 0;
    }

    public void update(XboxController driverController, XboxController operatorController){
        this.slowMode = driverController.getRightBumper();
        this.resetGyro = driverController.getBackButton();
        this.autoCollect = driverController.getAButton();
        this.robotOriented = driverController.getLeftBumper();
        this.score = driverController.getRightTriggerAxis()>0.1;
        this.partyMode = driverController.getStartButton();
        this.passAim = driverController.getYButton();
        this.forceShoot = driverController.getXButton();

        this.passThrough = operatorController.getRightTriggerAxis() >0.1;
        this.rangeTableShoot = operatorController.getRightBumper();
        this.shootFromPodium = operatorController.getBButton();
        this.outake = operatorController.getLeftBumper();
        this.intake = operatorController.getLeftTriggerAxis()>0.1;
        this.stow = operatorController.getAButton() || driverController.getBButton();
        this.amp = operatorController.getXButton();
        this.climb = operatorController.getYButton();
        this.manualExtend = operatorController.getPOV() == 0;
        this.manualRetract = operatorController.getPOV() == 180;
        this.ledAmpSignal = operatorController.getBackButton();
        this.trapPrime = operatorController.getStartButton();

        this.log(this.slowMode, "DriverController", "SlowMode");
        this.log(this.resetGyro, "DriverController", "ResetGyro");
        this.log(this.autoCollect, "DriverController", "AutoCollect");
        this.log(this.robotOriented, "DriverController", "RobotOriented");
        this.log(this.passThrough, "DriverController", "PassThrough");
        this.log(this.partyMode, "DriverController", "PartyMode");
        this.log(this.passAim, "DriverController", "PassAim");
        this.log(this.forceShoot, "DriverController", "ForceShoot");

        this.log(this.ledAmpSignal, "OperatorController", "LEDAmpSignal");
        this.log(this.shootFromPodium, "OperatorController", "ShootFromPodium");
        this.log(this.rangeTableShoot, "OperatorController", "RangeTableShot");
        this.log(this.outake, "OperatorController", "Outake");
        this.log(this.intake, "OperatorController", "Intake");
        this.log(this.stow, "OperatorController", "Stow");
        this.log(this.amp, "OperatorController", "Amp");
        this.log(this.climb, "OperatorController", "Climb");
        this.log(this.score, "OperatorController", "Score");
        this.log(this.manualExtend, "OperatorController", "ManualExtend");
        this.log(this.manualRetract, "OperatorController", "ManualRetract");
        this.log(this.kiddyPoolShot, "OperatorController", "JukeShot");
    }
    private void log(boolean logged, String controller, String name){
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name, logged);
    }
}
