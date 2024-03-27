package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

public class MainSubsystemsManager {
    public enum MechanismState {
        STOWED,
        STOWING,
        INTAKING,
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
        CLIMB
    }

    public MechanismState mechanismState = MechanismState.LOADED;
    public Intake intake;
    public Shooter shooter;
    public Elevator elevator;
    private Timer shootTimer = new Timer();
    private NeoPixelLED leds;

    public MainSubsystemsManager(Intake intake, Shooter shooter, Elevator elevator, NeoPixelLED leds) {
        this.intake = intake;
        this.shooter = shooter;
        this.elevator = elevator;
        this.leds = leds;
    }

    public void runMechanismStateMachine(Controls userControls, RangeTable.RangeEntry userRange){

        if(userControls.stow.isRisingEdge()){
            this.mechanismState = MechanismState.STOWING;
        }

        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"StateBeforeUpdate", this.mechanismState.toString());

        switch(this.mechanismState){

            // Stop the spinning motors and put the elevator in the home position

            case STOWING:
                elevator.setElevatorPosition(0, 0);
                shooter.stopFlywheels();
                shooter.stopFeeders();
                intake.spinPercentOutput(0);
                if(elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.STOWED;
                }
                break;


            // Does nothing; assumes STOWING has put everything in the robot in the home position

            case STOWED:
                //TODO: NOTIFY DRIVERS
                if(userControls.intake.isRisingEdge()){
                    this.mechanismState = MechanismState.INTAKING;
                }
                else if(userControls.climb.isRisingEdge()){
                    this.mechanismState = MechanismState.CLIMB_PRIME;
                }
                else if (userControls.outake.isRisingEdge()){
                    this.mechanismState = MechanismState.OUTTAKING;
                }
                else if (!elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.STOWING;
                }
                break;


            // Gets the note to the flywheels from the ground. TODO make sure we don't need the extra timer we had in the old shooter version of this

            case INTAKING:
                intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
                if(!shooter.isBottomBeamBreakTripped()){
                    shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED);
                }
                else{
                    shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 1); // The note will provide some resistance, so use the more powerful PID controller in slot 1
                }
                shooter.setShootVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED, SHOOTER.INTAKE_FLYWHEEL_SPEED);
                if(shooter.isBottomBeamBreakTripped()){
                    this.mechanismState = MechanismState.ADJUSTING_1;
                } 
                break;


            // First adjustment phase after intaking that positions the note just above the top beam-break.

            case ADJUSTING_1:
                shooter.setShootVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED, SHOOTER.ALIGN_FLYWHEEL_SPEED);
                shooter.setFeederVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
                if(shooter.isTopBeamBreakTripped()){
                    this.mechanismState = MechanismState.ADJUSTING_2;
                }
                break;


            // Second adjustment phase after intaking; positions the note in its final position just below the top beam-break

            case ADJUSTING_2:
                shooter.setShootVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED, SHOOTER.ALIGN_FLYWHEEL_SPEED);
                shooter.feederMotor.setVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
                if(!shooter.isTopBeamBreakTripped()){ //if beam break NOT tripped, exclamation point
                    this.mechanismState = MechanismState.LOADED;
                }
                break;


            // "Stow" state, but for when we have a note

            case LOADED:
                //TODO: NOTIFY DRIVERS
                if(userControls.outake.isRisingEdge()){
                    this.mechanismState = MechanismState.OUTTAKING;
                }
                else if(userControls.climb.isRisingEdge()){
                    this.mechanismState = MechanismState.CLIMB_PRIME;
                }
                else if(userControls.shootFromPodium.isRisingEdge()
                        || userControls.score.isRisingEdge() // For a subwoofer shot
                        || userControls.jukeShot.isRisingEdge()
                        || userControls.rangeTableShoot.isRisingEdge()){
                    this.mechanismState = MechanismState.PRIMING;
                }
                else if(userControls.amp.isRisingEdge()){
                    this.mechanismState = MechanismState.PRIMING_AMP;
                }
                break;


            // Regurgitate the note

            case OUTTAKING:
                intake.setIntakeVelocity(INTAKE.OUTAKE_VELOCITY);
                shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
                shooter.setShootVelocity(SHOOTER.OUTAKE_FLYWHEEL_SPEED, SHOOTER.OUTAKE_FLYWHEEL_SPEED);
                if(userControls.outake.isFallingEdge()){
                    this.mechanismState = MechanismState.STOWING;
                }
                break;


            // Prepare for a shot by spinning up the flywheels and angling the pivot

            case PRIMING:
                shooter.setTargetSpeed((int)userRange.leftFlywheelSpeed, (int)userRange.rightFlywheelSpeed);
                elevator.setElevatorPosition(userRange.pivotAngle, 0);
                if(shooter.readyToShoot() && elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.PRIMED;
                }
                break;


            // Wait for a score press or for something to cause us to be out of tolerance on the flywheel speeds or elevator
            // position (usually a change in the targets during a range table shot)

            case PRIMED:
                // Notify Drivers
                shootTimer.reset(); //Make sure the timer is running but always at zero until we shoot
                shootTimer.start();
                if(userControls.score.isRisingEdge()){
                    this.mechanismState = MechanismState.SHOOTING;
                } else if(!shooter.readyToShoot() || !elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.PRIMING;
                }
                break;


            // Under the assumption that our flywheels and pivot angle are set correctly, shoot into the speaker

            case SHOOTING:
                shooter.setFeederPower(SHOOTER.SHOOTING_FEEDER_POWER);
                if(userControls.score.isFallingEdge()){
                    this.mechanismState = MechanismState.STOWING;
                }
                if(shootTimer.hasElapsed(SHOOTER.SHOOT_SCORE_TIME)){
                    this.mechanismState = MechanismState.STOWING;
                }
                break;


            // Get into the amp elevator position (max extension and mid-height pivot)

            case PRIMING_AMP:
                shooter.stopFlywheels();
                elevator.setElevatorPosition(ELEVATOR.PIVOT_ANGLE_AMP, ELEVATOR.EXTENSION_METERS_AMP);
                if(elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.AMP_PRIMED;
                }
                break;


            // Static state that effectively waits for a score button press

            case AMP_PRIMED:
                // Notify Drivers
                if(userControls.score.isRisingEdge()){
                    this.mechanismState = MechanismState.AMP_SCORING;
                }
                break;


            // Score in the amp (assumes that we're in the amp position)

            case AMP_SCORING:
                shooter.setShootVelocity(SHOOTER.AMP_FLYWHEEL_SPEED, SHOOTER.AMP_FLYWHEEL_SPEED);
                shooter.setFeederPower(SHOOTER.AMP_FEEDER_SPEED);
                if(!userControls.score.isFallingEdge()){
                    this.mechanismState = MechanismState.AMP_PRIMED;
                }
                break;


            //Lift the pivot and extend to the maximum position

            case CLIMB_PRIME:
                shooter.stopFlywheels();
                elevator.setElevatorPosition(ELEVATOR.PIVOT_ANGLE_CLIMB, ELEVATOR.EXTENSION_METERS_CLIMB);
                if(elevator.isAtTargetPosition()){
                    this.mechanismState = MechanismState.CLIMB;
                }
                break;


            //It allows the user to manually extend and retract the elevator

            case CLIMB:
                if(userControls.manualExtend.isPressed()){
                    elevator.extend();
                } 
                else if(userControls.manualRetract.isPressed()){
                    elevator.retract();
                }
                break;

        }
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH+"StateAfterUpdate", this.mechanismState.toString());
    }

    public void setState(MechanismState state){
        if(DriverStation.isAutonomous()){ // To avoid any possible breakages, make sure this works exclusively in Auto (the only place it should be used)
            this.mechanismState = state;
        }
    }
}
