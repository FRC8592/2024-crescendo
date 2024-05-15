package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;
import frc.robot.NeoPixelLED.NewtonColor;
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
        PRIMING,
        PRIMED,
        SHOOTING,
        PRIMING_AMP,
        AMP_PRIMED,
        AMP_SCORING,
        CLIMB_PRIME,
        CLIMB,
        PASS_THROUGH_PRIME,
        PASS_THROUGH_1,
        PASS_THROUGH_2
    }

    public MechanismState mechanismState = MechanismState.LOADED;
    public Intake intake;
    public Shooter shooter;
    public Elevator elevator;
    private Timer shootTimer = new Timer();
    private Timer intakeTimer = new Timer(); // When this times-out, the LEDs will start complaining that the intake is taking too long.
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
        if (cameraRange == -1 || !useVision) {
            aimed = true; //assume it's aimed
        } else {
            aimed = targetLocked;
        }

        if(shooter.isMiddleBeamBreakTripped()){
            leds.solidColor(LEDS.CYAN);
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
                break;


            // Does nothing; assumes STOWING has put everything in the robot in the home position

            case STOWED:

                if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIME;
                }
                else if (!elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.STOWING;
                }
                else if(desireShot(userControls)){
                    this.mechanismState = MechanismState.PRIMING;
                }
                else if(userControls.amp){
                    this.mechanismState = MechanismState.PRIMING_AMP;
                }
                break;


            // Gets the note to the flywheels from the ground. TODO make sure we don't need the extra timer we had in the old shooter version of this

            case INTAKING:
                shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 0); // Set PID to when note is disenganged
                shooter.setShootVelocity(0,0);

                if(shooter.isTopBeamBreakTripped()){
                    shooter.setFeederPower(0);
                    this.mechanismState = MechanismState.AMP_PRIMED;
                }

                break;


            // Prepare for a shot by spinning up the flywheels and angling the pivot

            case PRIMING:
                shooter.setShootVelocity((int) userRange.leftFlywheelSpeed, (int) userRange.rightFlywheelSpeed);
                // elevator.setElevatorPosition(userRange.pivotAngle, userRange.elevatorHeight);

                if (userControls.amp) {
                    this.mechanismState = MechanismState.PRIMING_AMP;
                }
                else if(shooter.readyToShoot() && elevator.isAtTargetPosition()) {
                    this.mechanismState = MechanismState.PRIMED;
                }
                else if(userControls.manualExtend){
                    elevator.extend();
                } 
                else if(userControls.manualRetract){
                    elevator.retract();
                }
                if(userControls.manualPivotIncrease){
                    elevator.pivotIncrease();
                } 
                else if(userControls.manualPivotDecrease){
                    elevator.pivotDecrease();
                }

                break;


            // Wait for a score press or for something to cause us to be out of tolerance on the flywheel speeds or elevator
            // position (usually a change in the targets during a range table shot)

            case PRIMED:
                shootTimer.reset(); //Make sure the timer is running but always at zero until we shoot
                shootTimer.start();

                // Redundant setting of speeds in case of manual overwriting by drivers
                shooter.setShootVelocity((int) userRange.leftFlywheelSpeed, (int) userRange.rightFlywheelSpeed);
                // elevator.setElevatorPosition(userRange.pivotAngle, userRange.elevatorHeight);

                if (userControls.amp) {
                    this.mechanismState = MechanismState.PRIMING_AMP;
                } else if(userControls.score){
                    this.mechanismState = MechanismState.SHOOTING;
                }
                else if(userControls.manualExtend){
                    elevator.extend();
                } 
                else if(userControls.manualRetract){
                    elevator.retract();
                }
                if(userControls.manualPivotIncrease){
                    elevator.pivotIncrease();
                } 
                else if(userControls.manualPivotDecrease){
                    elevator.pivotDecrease();
                }

                break;


            // Under the assumption that our flywheels and pivot angle are set correctly, shoot into the speaker

            case SHOOTING:
                shooter.setFeederPower(SHOOTER.SHOOTING_FEEDER_POWER);


                if(shootTimer.hasElapsed(SHOOTER.SHOOT_SCORE_TIME)){
                    this.mechanismState = MechanismState.PRIMING_AMP;
                }
                break;


            // Get into the amp elevator position (max extension and mid-height pivot)

            case PRIMING_AMP:
                shooter.stopFlywheels(); // redundancy
                elevator.setElevatorPosition(ELEVATOR.PIVOT_ANGLE_AMP, ELEVATOR.EXTENSION_METERS_AMP);

                if(desireShot(userControls) && !userControls.score) { //TODO: Clean up this patch
                    this.mechanismState = MechanismState.PRIMING;
                } else if(elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.AMP_PRIMED;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIME;
                }

                break;


            // Static state that effectively waits for a score button press

            case AMP_PRIMED:
                shooter.stopFlywheels();
                shooter.stopFeeders();

                leds.solidColor(LEDS.CYAN);

                if(userControls.score){
                    this.mechanismState = MechanismState.AMP_SCORING;
                }
                else if (desireShot(userControls)){
                    this.mechanismState = MechanismState.PRIMING;
                }
                else if(userControls.climb){
                    this.mechanismState = MechanismState.CLIMB_PRIME;
                }
                else if(userControls.intake){
                    this.mechanismState = MechanismState.INTAKING;
                }
                break;


            // Score in the amp (assumes that we're in the amp position)

            case AMP_SCORING:
                shooter.setShootVelocity(SHOOTER.AMP_FLYWHEEL_SPEED, SHOOTER.AMP_FLYWHEEL_SPEED);
                shooter.setFeederPower(SHOOTER.AMP_FEEDER_SPEED);


                if(!userControls.score){
                    this.mechanismState = MechanismState.AMP_PRIMED;

                }
                break;


            //Lift the pivot and extend to the maximum position

            case CLIMB_PRIME:
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
                if(userControls.manualPivotIncrease){
                    elevator.pivotIncrease();
                } 
                else if(userControls.manualPivotDecrease){
                    elevator.pivotDecrease();
                }


                break;

        }
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"UserRange/LeftSpeed", this.userRange.leftFlywheelSpeed);
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"UserRange/RightSpeed", this.userRange.rightFlywheelSpeed);
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"UserRange/PivotAngle", this.userRange.pivotAngle);
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"StateAfterUpdate", this.mechanismState.toString());
    }

    public boolean desireShot(Controls userControls) {
        return userControls.score || userControls.rangeTableShoot;
    }
    public void resetToStowed(){
        this.mechanismState = MechanismState.STOWED;
    }
    public void resetToLoaded(){
        this.mechanismState = MechanismState.LOADED;
    }
}
