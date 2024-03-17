package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

public class MainSubsystemsManager {
    public enum MainStates {
        HOME,    // Everything goes to the home position
        SPEAKER, // Interact with the speaker
        AMP,     // Interact with the amp
        CLIMB,   // Interact with the stage
        INTAKE,  // Intake from the floor
        OUTAKE   // Get rid of anything we're holding
                 // Maybe add a case for source intaking.
    }

    public enum SubStates {
        INIT,   // All modes except HOME: runs this for a single frame before moving on to the normal control loop
        PREP,   // Score modes & Intake: get everything ready
        READY,  // Score modes: Continue staying ready, but signal that we can score
        SCORE,  // Score modes: Score the note
        UP,     // Climb: Move up
        DOWN,   // Climb: Move down.
        INTAKE, // Intake: run the intake and feeder motors
        SPINNING_UP, // Amp
        NOTHING
    }

    public MainStates mainState = MainStates.HOME;
    public SubStates subState = SubStates.NOTHING;
    private Intake intake;
    private Shooter shooter;
    private Elevator elevator;
    private Timer timer = new Timer(); //Used for shooting timeouts
    private PIDController turnToSpeakerController; // Will be used later for auto-aiming at the speaker.

    public MainSubsystemsManager(Intake intake, Shooter shooter, Elevator elevator) {
        this.intake = intake;
        this.shooter = shooter;
        this.elevator = elevator;
    }

    /*
     * GENERAL RULES FOR WRITING CODE WITH THE NEW LOGIC:
     *
     *
     * 1. Everything related to controls goes in Robot.java. MainSubsystemsManager assumes that whatever input it recieves is valid (although it shouldn't assume that the input doesn't ask the robot to break rules or mess something up)
     * 
     * 2. For cases where we can pysically run something but shouldn't (e.g. stowing while scoring in the amp), there should be a check in the method that sets the state.
     * 
     * 3. All code in the main state machine (`update()`) assumes that it is allowed to run and depends on the state-setters for that assumption. There shouldn't be any "are we allowed to do this?" checks there.
     *
     * 4. All main states have an INIT state that run initialization code and then switches the sub-state to PREP; all state-setters other than HOME set the sub-state to INIT.
     *
     * 6. All state-setters return whether they set the state as requested.
     */
    /**
     * Update the state machine for the shooter, elevator, and intake
     * @param distanceToSpeaker Distance from the speaker tag in inches from vision. This can be anything if not shooting into the speaker with a range table.
     * @param originalSpeeds ChassisSpeeds object derived exclusively from the controllers. {@code update()} will modify this as needed for the current function, then return it.
     * @return {@code originalSpeeds}, with any needed modifications for the current function.
     */
    public ChassisSpeeds update(double rangeTableFlywheels, double rangeTablePivot, ChassisSpeeds originalSpeeds) {
        ChassisSpeeds modifiedSpeeds = null; // We check if this object is a `null` reference at the end of the loop. We return originalSpeeds if so; otherwise, we return modifiedSpeeds.
        switch (mainState) { // State machine for the general mode
            case HOME: // No break, so `HOME` runs `default` code
            default: // Stop all the wheels and stow the elevator
                elevator.stow();
                stopWheels();
                timer.stop();
                timer.reset();
                break;
            case SPEAKER:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING; //If we're not in a compatible sub-state, set back to NOTHING and don't run anything else.
                        break;
                    case INIT: //Single-frame initialization case
                        intake.spinPercentOutput(0);
                        subState = SubStates.PREP;
                        shooter.shoot();
                        timer.stop();
                        timer.reset();
                        break;
                    case SCORE: // Note that this runs READY and PREP's code too because there's no `break;` after this code
                        timer.start();
                        shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED);
                        if (timer.get() > SHOOTER.SHOOT_SCORE_TIME && false /*never stop automatically*/) { //If we have spent enough time shooting
                            timer.stop();
                            timer.reset();
                            shooter.hasNote = false; //Make sure the "can't intake after scoring" bug doesn't come back
                            mainState = MainStates.HOME;
                            subState = SubStates.NOTHING;
                            break;
                        }
                    case READY: //This does nothing for now; TODO make this autodetect if the vision system fails and go back to PREP
                    case PREP:
                        shooter.setTargetSpeed((int)rangeTableFlywheels, (int)rangeTableFlywheels);
                        elevator.setPivotAngleCustom(rangeTablePivot);
                        if (shooter.readyToShoot() && elevator.isTargetAngle() && subState == SubStates.PREP/*TODO add a "vision working" condition*/) { // Remember, this is run in the SCORE case as well. We don't want to reset to READY while 
                            subState = SubStates.READY;                                                                                             // scoring, so we make sure we're running in the PREP case before setting to READY.
                        }
                        break;
                }
                break;
            case AMP:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING;
                        break;
                    case INIT:
                        stopWheels();
                        subState = SubStates.PREP;
                        timer.stop();
                        timer.reset();
                        break;
                    case SCORE: //Note that this runs READY and PREP's code too
                        if(Math.abs(shooter.feederMotor.getVelocity()-SHOOTER.AMP_FEEDER_SPEED)<SHOOTER.FEEDER_AMP_TOLERANCE){
                            timer.start();
                        }
                        shooter.setFeederVelocity(SHOOTER.AMP_FEEDER_SPEED);
                        shooter.setShootVelocity(SHOOTER.AMP_FLYWHEEL_SPEED, SHOOTER.AMP_FLYWHEEL_SPEED);
                        if (timer.get() > SHOOTER.AMP_SCORE_TIME) { //If we've been pushing the note into the amp for long enough
                            timer.stop();
                            timer.reset();
                            shooter.hasNote = false;
                            // mainState = MainStates.HOME;
                            subState = SubStates.INIT;
                            break;
                        }
                    case READY:
                    case PREP:
                        elevator.ampPosition();
                        if (elevator.isTargetAngle() && subState == SubStates.PREP) {
                            subState = SubStates.READY;
                        }
                        break;
                }
                break;
            case CLIMB:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING;
                        break;
                    case INIT:
                        stopWheels();
                        elevator.climbPosition();
                        timer.stop();
                        timer.reset();
                        subState = SubStates.PREP;
                        break;
                    case UP: // In this state when the buton is held
                        elevator.extend();
                        break;
                    case DOWN: // In this state when the buton is held
                        elevator.retract();
                        break;
                    case PREP:
                        if (elevator.isTargetAngle()) {
                            subState = SubStates.NOTHING;
                        }
                        break;
                }
                break;
            case INTAKE:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING;
                        break;
                    case INIT:
                        shooter.setShootPower(0); // Not using stopWheels here because we want to run the intake and feeder wheels
                        subState = SubStates.PREP;
                        timer.reset();
                        break;
                    case INTAKE:
                        intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
                        break;
                    case PREP:
                        elevator.stow();
                        if (elevator.isTargetAngle()) {
                            shooter.intake(); // Note that state HOME cancels this, so there's no need to worry about the feeders running when they're not supposed to
                            subState = SubStates.INTAKE;
                        }
                        break;
                }
                break;
            case OUTAKE:
                intake.setIntakeVelocity(INTAKE.OUTAKE_VELOCITY);
                shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
                shooter.setShootVelocity(SHOOTER.OUTAKE_FLYWHEEL_SPEED, SHOOTER.OUTAKE_FLYWHEEL_SPEED);
                break;
        }
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH + "MainState", mainState.name());
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH + "SubState", subState.name());
        return modifiedSpeeds == null ? originalSpeeds : modifiedSpeeds;
    }

    private void stopWheels() { // Stops the flywheels, feeder wheels, and intake wheels
        shooter.intakeStop();
        shooter.setFeederPower(0);
        shooter.setShootPower(0);
        intake.spinPercentOutput(0);
    }
    
    /**
     * If we're starting, stow and run the intake motors until the note is
     * collected.<p>
     * 
     * NOTE: If we stop the intake, it will only do anything if we are
     * currently intaking. If we're doing anything else, it will be ignored.
     * 
     * @param startStop a boolean for whether to start or stop the intake.
     * @return whether or not the state was set as requested.
     */
    public boolean intake(boolean startStop) {
        if (startStop) {
            if (mainState != MainStates.INTAKE) {
                mainState = MainStates.INTAKE;
                subState = SubStates.INIT;
                return true;
            }
        } else {
            if (mainState == MainStates.INTAKE) {
                mainState = MainStates.HOME;
                subState = SubStates.NOTHING;
                return true;
            }
        }
        return false;
    }

    /**
     * If we're in a mode that scores and we're ready to score, then score.
     *
     * @return whether or not the state was set as requested.
     */
    public boolean score() {
        if (subState == SubStates.READY) {
            subState = SubStates.SCORE;
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Go to amp position.
     * 
     * @param startStop start or stop amp position (stopping stows by default).
     */
    public boolean amp(boolean startStop) {
        if (startStop) {
            if (mainState != MainStates.AMP) { // If we're actively scoring, finish that score before continuing.
                mainState = MainStates.AMP;
                subState = SubStates.INIT;
                return true;
            }
        } else {
            if (mainState == MainStates.AMP) {
                mainState = MainStates.HOME;
                subState = SubStates.NOTHING;
                return true;
            }
        }
        return false;
    }

    /**
     * Start/stop preparing to shoot into the speaker
     * @param startStop whether to start or stop preparing
     */
    public boolean speaker(boolean startStop) {
        if (startStop) {
            if (mainState != MainStates.SPEAKER) {
                mainState = MainStates.SPEAKER;
                subState = SubStates.INIT;
                return true;
            }
        } else {
            if (mainState == MainStates.SPEAKER) {
                mainState = MainStates.HOME;
                subState = SubStates.NOTHING;
                return true;
            }
        }
        return false;
    }

    /**
     * Return all subsystems to the home positions/speeds. Does nothing if actively scoring.
     */
    public void home() {
        mainState = MainStates.HOME;
        subState = SubStates.NOTHING;
    }

    /**
     * Start/stop preparing for a climb
     * 
     * @param startStop whether to start (lift the pivot and extension to the max) or stop (stow) preparing. Note that stopping doesn't do anything if not already in the climbing state.
     */
    public boolean climb(boolean startStop){
        if (startStop) {
            if (mainState != MainStates.CLIMB) {
                mainState = MainStates.CLIMB;
                subState = SubStates.INIT;
                return true;
            }
        } else {
            if (mainState == MainStates.CLIMB) {
                mainState = MainStates.HOME;
                subState = SubStates.NOTHING;
                return true;
            }
        }
        return false;
    }

    /**
     * Start or stop extending the elevator. Respects soft limits.
     * @param startStop start or stop extending. Note that this does nothing if the pivot is not at full height.
     */
    public void climbExtend(boolean startStop) {
        if (mainState == MainStates.CLIMB && subState != SubStates.PREP) { //In climb mode, but done preparing.
            if (startStop) {
                if (subState != SubStates.UP) {
                    subState = SubStates.UP;
                }
            }
            else {
                if (subState == SubStates.UP) {
                    subState = SubStates.NOTHING;
                }
            }
        }
    }

    /**
     * Start or stop retracting the elevator. Respects soft limits.
     * 
     * @param startStop start or stop retracting. Note that this does nothing if the pivot is not at full height.
     */
    public void climbRetract(boolean startStop) {
        if (mainState == MainStates.CLIMB && subState != SubStates.PREP) { // In climb mode, but done preparing.
            if (startStop) {
                if (subState != SubStates.DOWN) {
                    subState = SubStates.DOWN;
                }
            } else {
                if (subState == SubStates.DOWN) {
                    subState = SubStates.NOTHING;
                }
            }
        }
    }

    /**
     * Start or stop outaking. Does nothing if actively scoring.
     * @param startStop
     */
    public void outake(boolean startStop) {
        if (subState != SubStates.SCORE) {
            if (startStop) {
                if (mainState != MainStates.OUTAKE) {
                    mainState = MainStates.OUTAKE;
                    subState = SubStates.NOTHING;
                }
            } else {
                if (mainState == MainStates.OUTAKE) {
                    mainState = MainStates.HOME;
                    subState = SubStates.NOTHING;
                }
            }
        }
    }
}
