package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

public class MainSubsystemsManager {
    private enum MainStates {
        HOME, // Everything goes to the home position
        SPEAKER, // Interact with the speaker
        AMP, //Interact with the amp
        CLIMB, // Interact with the stage
        INTAKE, // Intake from the floor
        OUTAKE //Get rid of anything we're holding
        // Maybe add a case for source intaking.
    }

    private enum SubStates {
        PREP,   // Score modes & Intake: get everything ready
        READY,  // Score modes: Continue staying ready, but signal that we can score
        SCORE,  // Score modes: Score the note
        UP,     // Climb: Move up
        DOWN,   // Climb: Move down.
        INTAKE, // Intake: run the intake and feeder motors
        NOTHING
    }

    private MainStates mainState = MainStates.HOME;
    private SubStates subState = SubStates.NOTHING;
    private Intake intake;
    private Shooter shooter;
    private Elevator elevator;
    private Timer timer = new Timer();
    private PIDController turnToSpeakerController;

    public MainSubsystemsManager(Intake intake, Shooter shooter, Elevator elevator) {
        this.intake = intake;
        this.shooter = shooter;
        this.elevator = elevator;
    }

    /**
     * Update the state machine for the shooter, elevator, and intake
     * @param distanceToSpeaker Distance from the speaker tag in inches from vision. This can be anything if not shooting into the speaker with a range table.
     * @param originalSpeeds ChassisSpeeds object derived exclusively from the controllers. {@code update()} will modify this as needed for the current function, then return it.
     * @return {@code originalSpeeds}, with any needed modifications for the current function.
     */
    public ChassisSpeeds update(double distanceToSpeaker, ChassisSpeeds originalSpeeds) {
        ChassisSpeeds modifiedSpeeds = null;
        switch (mainState) {
            case HOME:
            default: // Stop all the wheels and stow the elevator
                elevator.stow();
                shooter.setFeederSpeed(0);
                shooter.setShootPercentOutput(0);
                intake.spinPercentOutput(0);
                break;
            case SPEAKER:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING; //If we're not in a compatible sub-state, set back to NOTHING and don't run anything else.
                        break;
                    case SCORE: // Note that this runs READY and PREP's code too because there's no `break;` after this code
                        timer.start();
                        shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED);
                        if (timer.get() > SHOOTER.SHOOT_SCORE_TIME) {
                            timer.stop();
                            timer.reset();
                            shooter.hasNote = false;
                            mainState = MainStates.HOME;
                            subState = SubStates.NOTHING;
                            break;
                        }
                    case READY: //Does the same thing as PREP; this is just a flag for the score() function
                    case PREP:
                        RangeTable.RangeEntry target = RangeTable.get(distanceToSpeaker);
                        shooter.setShootVelocity(target.flywheelSpeed, target.flywheelSpeed);
                        elevator.setPivotAngleCustom(target.pivotAngle);
                        if (shooter.isReady() && elevator.isTargetAngle() && subState != SubStates.SCORE) { // The second condition is in case we're SCOREing as well as PREPping. We don't want to reset to READY while scoring.
                            subState = SubStates.READY;
                        }
                        break;
                }
                break;
            case AMP:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING;
                        break;
                    case SCORE: //Note that this runs READY and PREP's code too
                        timer.start();
                        shooter.setFeederVelocity(SHOOTER.AMP_FEEDER_SPEED);
                        shooter.setShootVelocity(SHOOTER.AMP_FLYWHEEL_SPEED, SHOOTER.AMP_FLYWHEEL_SPEED);
                        if (timer.get() > SHOOTER.AMP_SCORE_TIME) {
                            timer.stop();
                            timer.reset();
                            shooter.hasNote = false;
                            mainState = MainStates.HOME;
                            subState = SubStates.NOTHING;
                            break;
                        }
                    case READY:
                    case PREP:
                        elevator.setPivotAngleCustom(ELEVATOR.PIVOT_ANGLE_AMP);
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
                    case UP:
                        elevator.extend();
                        break;
                    case DOWN:
                        elevator.retract();
                        break;
                    case PREP:
                        elevator.climbPosition();
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
                    case INTAKE:
                        intake.spinPercentOutput(INTAKE.INTAKE_POWER);
                        shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED);
                        if (shooter.hasNote()) {
                            mainState = MainStates.HOME;
                            subState = SubStates.NOTHING;
                        }
                        break;
                    case PREP:
                        elevator.stow();
                        if (elevator.isTargetAngle()) {
                            subState = SubStates.INTAKE;
                        }
                        break;
                }
                break;
            case OUTAKE:
                intake.spinPercentOutput(INTAKE.OUTAKE_POWER);
                shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
                shooter.setShootVelocity(SHOOTER.OUTAKE_FLYWHEEL_SPEED, SHOOTER.OUTAKE_FLYWHEEL_SPEED);
                break;
        }
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH + "MainState", mainState.name());
        Logger.recordOutput(MAIN_SUBSYSTEMS_MANAGER.LOG_PATH + "SubState", subState.name());
        return modifiedSpeeds == null ? originalSpeeds : modifiedSpeeds;
    }

    /**
     * If we're starting, stow and run the intake motors until the note is
     * collected.<p>
     * 
     * NOTE: If we stop the intake, it will only do anything if we are
     * currently intaking. If we're doing anything else, it will be ignored.
     * 
     * @param startStop a boolean for whether to start or stop the intake.
     */
    public void intake(boolean startStop) {
        if (subState != SubStates.SCORE && !shooter.hasNote()) {
            if (startStop) {
                if (mainState != MainStates.INTAKE) {
                    mainState = MainStates.INTAKE;
                    subState = SubStates.PREP;
                }
            } else {
                if (mainState == MainStates.INTAKE) {
                    mainState = MainStates.HOME;
                    subState = SubStates.NOTHING;
                }
            }
        }
    }

    /**
     * If we're in a mode that scores and we're ready to score, then score.
     */
    public void score() {
        if (subState == SubStates.READY) {
            subState = SubStates.SCORE;
        }
    }

    /**
     * Go to amp position.
     * 
     * @param startStop start or stop amp position (stopping stows by default).
     */
    public void amp(boolean startStop) {
        if (subState != SubStates.SCORE) {
            if (startStop) {
                if (mainState != MainStates.AMP) { // If we're actively scoring, finish that score before continuing.
                    mainState = MainStates.AMP;
                    subState = SubStates.PREP;
                }
            } else {
                if (mainState == MainStates.AMP) {
                    mainState = MainStates.HOME;
                    subState = SubStates.NOTHING;
                }
            }
        }
    }

    /**
     * Start/stop preparing to shoot into the speaker
     * @param startStop whether to start or stop preparing
     */
    public void speaker(boolean startStop) {
        if (subState != SubStates.SCORE) {
            if (startStop) {
                if (mainState != MainStates.SPEAKER) {
                    mainState = MainStates.SPEAKER;
                    subState = SubStates.PREP;
                }
            } else {
                if (mainState == MainStates.SPEAKER) {
                    mainState = MainStates.HOME;
                    subState = SubStates.NOTHING;
                }
            }
        }
    }

    /**
     * Return all subsystems to the home positions/speeds. Does nothing if actively scoring.
     */
    public void home() {
        if (subState != SubStates.SCORE) {
            mainState = MainStates.HOME;
            subState = SubStates.NOTHING;
        }
    }

    /**
     * Return all subsystems to the home positions/speeds. Overrides the usual "ignore-if-scoring" rule
     */
    public void forceHome() {
        mainState = MainStates.HOME;
        subState = SubStates.NOTHING;
    }

    /**
     * Start/stop preparing for a climb
     * 
     * @param startStop whether to start (lift the pivot and extension to the max) or stop (stow) preparing. Note that stopping doesn't do anything if not already in the climbing state.
     */
    public void climb(boolean startStop){
        if (subState != SubStates.SCORE) {
            if (startStop) {
                if (mainState != MainStates.CLIMB) {
                    mainState = MainStates.CLIMB;
                    subState = SubStates.PREP;
                }
            } else {
                if (mainState == MainStates.CLIMB) {
                    mainState = MainStates.HOME;
                    subState = SubStates.NOTHING;
                }
            }
        }
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
                    mainState = MainStates.OUTAKE;
                    subState = SubStates.NOTHING;
                }
            }
        }
    }
}
