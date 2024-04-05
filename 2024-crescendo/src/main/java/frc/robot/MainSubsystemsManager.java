package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;
import frc.robot.RangeTable.RangeEntry;

public class MainSubsystemsManager {
    public enum MechanismState {
        STOWED,
        STOWING,
        INTAKING,
        INTAKING_2,
        ADJUSTING_1,
        ADJUSTING_2,
        LOADED,
        OUTTAKING,
        SHOOT_PRIMING,
        SHOOT_PRIMED,
        SHOOT_SCORING,
        AMP_PRIMING,
        AMP_PRIMED,
        AMP_SCORING,
        CLIMB_PRIMING,
        CLIMB,
        PASS_THROUGH_1,
        PASS_THROUGH_2
    }

    public MechanismState mechanismState = MechanismState.LOADED;
    public Intake intake;
    public Shooter shooter;
    public Elevator elevator;
    private Timer shootTimer = new Timer();
    private NeoPixelLED leds;
    private RangeEntry userRange = new RangeEntry(0, 0, 0);
    private boolean useVision = true;
    private boolean aimed = true;

    public MainSubsystemsManager(Intake intake, Shooter shooter, Elevator elevator, NeoPixelLED leds) {
        this.intake = intake;
        this.shooter = shooter;
        this.elevator = elevator;
        this.leds = leds;
    }

    //
    // static range
    //
    public void staticPrime(RangeEntry staticRange) {
        this.userRange = staticRange;
        this.useVision = false;
    }


    //
    //dynamic (vision) range
    //
    public void setVisionPrime() {
        this.useVision = true;
    }

    public void updateMechanismStateMachine(Controls userControls, double cameraRange, boolean targetLocked){

        if(userControls.stow){
            this.mechanismState = MechanismState.STOWING;
        }

        // If we are using vision and if we can see the Apriltag, overwrite our last rangeTable value with a new value based on distance
        if (useVision && cameraRange != -1) {
            userRange = RangeTable.get(cameraRange);
        }

        // TODO: This may shoot prematurely if we lose sight of the AprilTag while aiming
        if (cameraRange == -1) {
            aimed = true; //assume it's aimed
        } else {
            aimed = targetLocked;
        }

        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"StateBeforeUpdate", this.mechanismState.toString());

        switch(this.mechanismState){

            // Stop the spinning motors and put the elevator in the home position

            case STOWING:
                elevator.setElevatorPosition(0, 0);
                shooter.stopFlywheels();
                shooter.stopFeeders();
                intake.stopIntake();
                if(elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.STOWED;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }
                else if (userControls.amp) {
                    this.mechanismState = MechanismState.AMP_PRIMING;
                }
                else if (desireShot(userControls)) {
                    this.mechanismState = MechanismState.SHOOT_PRIMING;
                }
                break;


            // Does nothing; assumes STOWING has put everything in the robot in the home position

            case STOWED:
                //TODO: NOTIFY DRIVERS
                if(userControls.intake){
                    this.mechanismState = MechanismState.INTAKING;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }
                else if (userControls.outake){
                    this.mechanismState = MechanismState.OUTTAKING;
                }
                else if (!elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.STOWING;
                }
                else if(desireShot(userControls)){
                    this.mechanismState = MechanismState.SHOOT_PRIMING;
                }
                else if(userControls.amp){
                    this.mechanismState = MechanismState.AMP_PRIMING;
                }
                else if(userControls.passThrough){
                    this.mechanismState = MechanismState.PASS_THROUGH_1;
                }
                else {
                    this.staticPrime(RangeTable.get(1.4));
                }
                break;


            // Gets the note to the flywheels from the ground. TODO make sure we don't need the extra timer we had in the old shooter version of this

            case INTAKING:
                intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
                shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 0); // Set PID to when note is disenganged
                // shooter.setFeederPower(1);
                shooter.setShootVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED, SHOOTER.INTAKE_FLYWHEEL_SPEED);

                // if(Rumble.isQueueEmpty(Rumble.Controller.OPERATOR)){
                //     Rumble.enqueueRumbleBump(Rumble.Controller.OPERATOR, new Rumble().new RumbleBump(0.1, 0.25));
                // }
                // if(Rumble.isQueueEmpty(Rumble.Controller.DRIVER)){
                //     Rumble.enqueueRumbleBump(Rumble.Controller.DRIVER, new Rumble().new RumbleBump(0.1, 0.25));
                // }

                if(shooter.isBottomBeamBreakTripped()){
                    this.mechanismState = MechanismState.INTAKING_2;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }

                break;
                
            // Continues intaking while bottom beam break is hit until note fully passes it

            case INTAKING_2:
                intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
                shooter.setFeederPower(1); // Set PID to when note is engaged
                shooter.setShootVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED, SHOOTER.INTAKE_FLYWHEEL_SPEED);

                // if(Rumble.isQueueEmpty(Rumble.Controller.OPERATOR)){
                //     Rumble.enqueueRumbleBump(Rumble.Controller.OPERATOR, new Rumble().new RumbleBump(0.1, 0.25));
                // }
                // if(Rumble.isQueueEmpty(Rumble.Controller.DRIVER)){
                //     Rumble.enqueueRumbleBump(Rumble.Controller.DRIVER, new Rumble().new RumbleBump(0.1, 0.25));
                // }

                if(!shooter.isBottomBeamBreakTripped()){
                    this.mechanismState = MechanismState.ADJUSTING_1;
                    intake.stopIntake();
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }

                break;

            // First adjustment phase after intaking that positions the note just above the top beam-break.

            case ADJUSTING_1:
                shooter.setShootVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED, SHOOTER.ALIGN_FLYWHEEL_SPEED);
                shooter.setFeederVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
                if(shooter.isTopBeamBreakTripped() && shooter.feederMotor.getVelocity() < 0){
                    this.mechanismState = MechanismState.ADJUSTING_2;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }
                break;


            // Second adjustment phase after intaking; positions the note in its final position just below the top beam-break

            case ADJUSTING_2:
                shooter.setShootVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED, SHOOTER.ALIGN_FLYWHEEL_SPEED);
                shooter.feederMotor.setVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
                if(!shooter.isTopBeamBreakTripped()){ //if beam break NOT tripped, exclamation point
                    this.mechanismState = MechanismState.LOADED;
                    // shooter.stopFeeders();
                    shooter.setFeederVelocity(0, 1);
                    shooter.setShootVelocity(0, 0);
                    // shooter.stopFlywheels();
                    //Rumble to signal that we have the note ready
                    Rumble.enqueueRumbleBump(Rumble.Controller.OPERATOR, new Rumble().new RumbleBump(0.3, 1));
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }
                break;


            // "Stow" state, but for when we have a note

            case LOADED:
                shooter.setShootVelocity(userRange.leftFlywheelSpeed, userRange.rightFlywheelSpeed);
                leds.notePickup();

                if(userControls.outake){
                    this.mechanismState = MechanismState.OUTTAKING;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }
                else if(desireShot(userControls)){
                    this.mechanismState = MechanismState.SHOOT_PRIMING;
                }
                else if(userControls.amp){
                    this.mechanismState = MechanismState.AMP_PRIMING;
                }
                else if(userControls.passThrough){
                    this.mechanismState = MechanismState.PASS_THROUGH_1;
                }
                else {
                    this.staticPrime(RangeTable.get(1.4));
                }
                break;


            // Regurgitate the note

            case OUTTAKING:
                intake.setIntakeVelocity(INTAKE.OUTAKE_VELOCITY);
                shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_SPEED, 2);
                shooter.setShootVelocity(SHOOTER.OUTAKE_FLYWHEEL_SPEED, SHOOTER.OUTAKE_FLYWHEEL_SPEED);

                if(!userControls.outake){ // Is NOT pressed
                    this.mechanismState = MechanismState.STOWING;
                }
                break;


            // Prepare for a shot by spinning up the flywheels and angling the pivot

            case SHOOT_PRIMING:
                shooter.setShootVelocity((int) userRange.leftFlywheelSpeed, (int) userRange.rightFlywheelSpeed);
                elevator.setElevatorPosition(userRange.pivotAngle, userRange.elevatorHeight);

                if (userControls.amp) {
                    this.mechanismState = MechanismState.AMP_PRIMING;
                } else if(shooter.readyToShoot() && elevator.isAtTargetPosition() && (aimed || isStaticShot(userRange))) { //For the last case, this means we don't care wether we're aimed if we're not range-table shooting
                    this.mechanismState = MechanismState.SHOOT_PRIMED;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }

                break;


            // Wait for a score press or for something to cause us to be out of tolerance on the flywheel speeds or elevator
            // position (usually a change in the targets during a range table shot)

            case SHOOT_PRIMED:
                shootTimer.reset(); //Make sure the timer is running but always at zero until we shoot
                shootTimer.start();

                // Redundant setting of speeds in case of manual overwriting by drivers
                shooter.setShootVelocity((int) userRange.leftFlywheelSpeed, (int) userRange.rightFlywheelSpeed);
                elevator.setElevatorPosition(userRange.pivotAngle, userRange.elevatorHeight);
                

                //Constantly rumble both controllers to let both drivers know that we're ready to shoot
                // if(Rumble.isQueueEmpty(Rumble.Controller.OPERATOR)){
                //     Rumble.enqueueRumbleBump(Rumble.Controller.OPERATOR, new Rumble().new RumbleBump(0.1, 0.25));
                // }
                // if(Rumble.isQueueEmpty(Rumble.Controller.DRIVER)){
                //     Rumble.enqueueRumbleBump(Rumble.Controller.DRIVER, new Rumble().new RumbleBump(0.1, 0.25));
                // }

                if (userControls.amp) {
                    this.mechanismState = MechanismState.AMP_PRIMING;
                } else if(!shooter.readyToShoot() || !elevator.isAtTargetPosition() || !(aimed || isStaticShot(userRange))){
                    this.mechanismState = MechanismState.SHOOT_PRIMING;
                } else if(userControls.score){
                    this.mechanismState = MechanismState.SHOOT_SCORING;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }

                break;


            // Under the assumption that our flywheels and pivot angle are set correctly, shoot into the speaker

            case SHOOT_SCORING:
                shooter.setFeederPower(SHOOTER.SHOOTING_FEEDER_POWER);
                if(shootTimer.hasElapsed(SHOOTER.SHOOT_SCORE_TIME)){
                    this.mechanismState = MechanismState.STOWING;
                    leds.off();
                }
                break;


            // Get into the amp elevator position (max extension and mid-height pivot)

            case AMP_PRIMING:
                shooter.stopFlywheels(); // redundancy
                elevator.setElevatorPosition(ELEVATOR.PIVOT_ANGLE_AMP, ELEVATOR.EXTENSION_METERS_AMP);

                if(desireShot(userControls) && !userControls.score) { //TODO: Clean up this patch
                    this.mechanismState = MechanismState.SHOOT_PRIMING;
                } else if(elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.AMP_PRIMED;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }

                break;


            // Static state that effectively waits for a score button press

            case AMP_PRIMED:
                shooter.stopFlywheels();
                shooter.stopFeeders();
                if(userControls.score){
                    this.mechanismState = MechanismState.AMP_SCORING;
                }
                else if (desireShot(userControls)){
                    this.mechanismState = MechanismState.SHOOT_PRIMING;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIMING;
                }
                break;


            // Score in the amp (assumes that we're in the amp position)

            case AMP_SCORING:
                shooter.setShootVelocity(SHOOTER.AMP_FLYWHEEL_SPEED, SHOOTER.AMP_FLYWHEEL_SPEED);
                shooter.setFeederPower(SHOOTER.AMP_FEEDER_SPEED);
                if(!userControls.score){
                    this.mechanismState = MechanismState.AMP_PRIMED;
                    leds.off();
                }
                break;


            //Lift the pivot and extend to the maximum position

            case CLIMB_PRIMING:
                shooter.stopFlywheels();
                shooter.stopFeeders();
                intake.stopIntake();
                elevator.setElevatorPosition(ELEVATOR.PIVOT_ANGLE_CLIMB, ELEVATOR.EXTENSION_METERS_CLIMB);
                if(elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.CLIMB;
                }
                break;


            //It allows the user to manually extend and retract the elevator

            case CLIMB:
                if(userControls.manualExtend){
                    elevator.extend();
                } 
                else if(userControls.manualRetract){
                    elevator.retract();
                }
                break;

            case PASS_THROUGH_1:
                intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
                    shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 0); // Set PID to when note is disenganged
                    // shooter.setFeederPower(1);
                    shooter.setShootVelocity(6000, 6000);

                    // if(Rumble.isQueueEmpty(Rumble.Controller.OPERATOR)){
                    //     Rumble.enqueueRumbleBump(Rumble.Controller.OPERATOR, new Rumble().new RumbleBump(0.1, 0.25));
                    // }
                    // if(Rumble.isQueueEmpty(Rumble.Controller.DRIVER)){
                    //     Rumble.enqueueRumbleBump(Rumble.Controller.DRIVER, new Rumble().new RumbleBump(0.1, 0.25));
                    // }

                    if(shooter.isBottomBeamBreakTripped()){
                        this.mechanismState = MechanismState.PASS_THROUGH_2;
                    }

                    break;
            case PASS_THROUGH_2:
                intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
                shooter.setFeederPower(1); // Set PID to when note is engaged
                shooter.setShootVelocity(6000, 6000);

                // if(Rumble.isQueueEmpty(Rumble.Controller.OPERATOR)){
                //     Rumble.enqueueRumbleBump(Rumble.Controller.OPERATOR, new Rumble().new RumbleBump(0.1, 0.25));
                // }
                // if(Rumble.isQueueEmpty(Rumble.Controller.DRIVER)){
                //     Rumble.enqueueRumbleBump(Rumble.Controller.DRIVER, new Rumble().new RumbleBump(0.1, 0.25));
                // }

                if(!shooter.isBottomBeamBreakTripped()){
                    this.mechanismState = MechanismState.STOWING;
                    intake.stopIntake();
                }

                break;

        }
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"UserRange/LeftSpeed", this.userRange.leftFlywheelSpeed);
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"UserRange/RightSpeed", this.userRange.rightFlywheelSpeed);
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"UserRange/PivotAngle", this.userRange.pivotAngle);
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"StateAfterUpdate", this.mechanismState.toString());
    }

    public boolean desireShot(Controls userControls) {
        return userControls.shootFromPodium
                        || userControls.score // For a subwoofer shot
                        || userControls.kiddyPoolShot
                        || userControls.rangeTableShoot;
    }
    public void resetToStowed(){
        this.mechanismState = MechanismState.STOWED;
    }
    public void resetToLoaded(){
        this.mechanismState = MechanismState.LOADED;
    }
    public boolean isStaticShot(RangeEntry entry){
        return entry.equals(RangeTable.getSubwoofer())
                || entry.equals(RangeTable.getPodium())
                || entry.equals(RangeTable.getKiddyPool());
    }
}
