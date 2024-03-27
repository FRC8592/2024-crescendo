package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;

public class Controls {
    public BooleanManager slowMode;
    public BooleanManager resetGyro;
    public BooleanManager autoCollect;
    public BooleanManager robotOriented;

    public BooleanManager ledAmpSignal;
    public BooleanManager shootFromPodium;
    public BooleanManager rangeTableShoot;
    public BooleanManager outake;
    public BooleanManager intake;
    public BooleanManager stow;
    public BooleanManager amp;
    public BooleanManager climb;
    public BooleanManager score;
    public BooleanManager manualExtend;
    public BooleanManager manualRetract;
    public BooleanManager jukeShot;

    public Controls(){
        this.slowMode = new BooleanManager(false);
        this.resetGyro = new BooleanManager(false);
        this.autoCollect = new BooleanManager(false);
        this.robotOriented = new BooleanManager(false);

        this.ledAmpSignal = new BooleanManager(false);
        this.shootFromPodium = new BooleanManager(false);
        this.rangeTableShoot = new BooleanManager(false);
        this.outake = new BooleanManager(false);
        this.intake = new BooleanManager(false);
        this.stow = new BooleanManager(false);
        this.amp = new BooleanManager(false);
        this.climb = new BooleanManager(false);
        this.score = new BooleanManager(false);
        this.manualExtend = new BooleanManager(false);
        this.manualRetract = new BooleanManager(false);
        this.jukeShot = new BooleanManager(false);
    }

    public void update(XboxController driverController, XboxController operatorController){
        this.slowMode.update       (driverController.getRightBumper());
        this.resetGyro.update      (driverController.getBackButton());
        this.autoCollect.update    (driverController.getLeftBumper());
        this.robotOriented.update  (driverController.getRightTriggerAxis() >0.1);

        this.shootFromPodium.update(operatorController.getLeftBumper());
        this.rangeTableShoot.update(operatorController.getBButton());
        this.outake.update         (operatorController.getRightBumper());
        this.intake.update         (operatorController.getLeftTriggerAxis()>0.1);
        this.stow.update           (operatorController.getAButton());
        this.amp.update            (operatorController.getXButton());
        this.climb.update          (operatorController.getYButton());
        this.score.update          (operatorController.getRightTriggerAxis()>0.1);
        this.manualExtend.update   (operatorController.getPOV() == 0);
        this.manualRetract.update  (operatorController.getPOV() == 180);
        this.ledAmpSignal.update   (operatorController.getBackButton());
        this.jukeShot.update       (operatorController.getStartButton());

        this.log(this.slowMode, "DriverController", "SlowMode");
        this.log(this.resetGyro, "DriverController", "ResetGyro");
        this.log(this.autoCollect, "DriverController", "AutoCollect");
        this.log(this.robotOriented, "DriverController", "RobotOriented");

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
        this.log(this.jukeShot, "OperatorController", "JukeShot");
    }
    private void log(BooleanManager logged, String controller, String name){
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name+"/IsPressed", logged.isPressed());
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name+"/IsRisingEdge", logged.isRisingEdge());
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name+"/IsFallingEdge", logged.isFallingEdge());
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name+"/IsToggled", logged.isToggle());
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name+"/IsToggleRisingEdge", logged.isToggleRisingEdge());
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name+"/isToggleFallingEdge", logged.isToggleFallingEdge());
        Logger.recordOutput(CONTROLS.LOG_PATH+controller+"/"+name+"/IsTriggered", logged.isTriggered());
    }
}
