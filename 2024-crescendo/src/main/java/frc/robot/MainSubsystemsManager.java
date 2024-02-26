package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

public class MainSubsystemsManager {
    private enum MainStates {
        HOME,
        SPEAKER,
        AMP,
        CLIMB,
        INTAKE
    }

    private enum SubStates {
        PREP,
        READY,
        SCORE,
        UP,
        DOWN,
        INTAKE,
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
     * @param distanceToSpeaker Distance from the speaker tag in inches from Manav's system. This can be anything if not shooting into the speaker with a range table.
     * @param originalSpeeds ChassisSpeeds object derived exclusively from the controllers. {@code update()} will modify this as needed for the current function, then return it.
     * @return {@code originalSpeeds}, with any needed modifications for the current function.
     */
    public ChassisSpeeds update(double distanceToSpeaker, ChassisSpeeds originalSpeeds) {
        switch (mainState) {
            case HOME:
            default:
                elevator.stow();
                shooter.setFeederSpeed(0);
                shooter.setShootPercentOutput(0);
                intake.spinPercentOutput(0);
                return originalSpeeds;
            case SPEAKER:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING; //If we're not in a compatible sub-state, set back to NOTHING.
                        return originalSpeeds;
                    case SCORE:
                        timer.start();
                        shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED);
                        if (timer.get() > SHOOTER.SHOOT_SCORE_TIME) {
                            timer.stop();
                            timer.reset();
                            mainState = MainStates.HOME;
                            subState = SubStates.NOTHING;
                            return originalSpeeds;
                        }
                    case PREP: //No break here, so SCORE also runs PREP's code
                        RangeTable.RangeEntry target = RangeTable.get(distanceToSpeaker);
                        shooter.setShootVelocity(target.flywheelSpeed, target.flywheelSpeed);
                        elevator.setPivotAngleCustom(target.pivotAngle);
                        if (shooter.isReady() && elevator.isTargetAngle() && subState != SubStates.SCORE) { // The second condition is in case we're SCOREing as well as PREPping. We don't want to reset to READY while scoring.
                            subState = SubStates.READY;
                        }
                        return originalSpeeds;
                }
            case AMP:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING;
                        return originalSpeeds;
                    case SCORE:
                        timer.start();
                        shooter.setFeederVelocity(SHOOTER.AMP_FEEDER_SPEED);
                        shooter.setShootVelocity(SHOOTER.AMP_SHOOTER_SPEED, SHOOTER.AMP_SHOOTER_SPEED);
                        if (timer.get() > SHOOTER.AMP_SCORE_TIME) {
                            timer.stop();
                            timer.reset();
                            mainState = MainStates.HOME;
                            subState = SubStates.NOTHING;
                            return originalSpeeds;
                        }
                    case PREP:
                        elevator.setPivotAngleCustom(ELEVATOR.PIVOT_ANGLE_AMP);
                        if (elevator.isTargetAngle() && subState != SubStates.SCORE) {
                            subState = SubStates.READY;
                        }
                        return originalSpeeds;
                }
            case CLIMB:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING;
                        return originalSpeeds;
                    case PREP:
                        elevator.climbPosition();
                        if (elevator.isTargetAngle()) {
                            subState = SubStates.NOTHING;
                        }
                    case UP:
                        elevator.extend();
                        return originalSpeeds;
                    case DOWN:
                        elevator.retract();
                        return originalSpeeds;
                }
            case INTAKE:
                switch (subState) {
                    default:
                        subState = SubStates.NOTHING;
                        return originalSpeeds;
                    case PREP:
                        elevator.stow();
                        if (elevator.isTargetAngle()) {
                            subState = SubStates.INTAKE;
                        }
                        return originalSpeeds;
                    case INTAKE:
                        intake.spinPercentOutput(INTAKE.INTAKE_POWER);
                        shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED);
                        if (shooter.hasNote()) {
                            mainState = MainStates.HOME;
                            subState = SubStates.NOTHING;
                        }
                        return originalSpeeds;
                }
        }
    }

    /**
     * Stow, then run the intake motors until the note is collected.
     * @param yn a boolean for whether to start or stop the intake. Intended to be used with rising- and falling-edge triggers on the controller.
     */
    public void intake(boolean yn) {
        if (yn) {
            if (mainState != MainStates.INTAKE) { // subState could be something other than PREP, so this blocks it from being reset.
                mainState = MainStates.INTAKE;
                subState = SubStates.PREP;
            }
        }
        else {
            if (mainState == MainStates.INTAKE) {
                mainState = MainStates.HOME;
                subState = SubStates.NOTHING;
            }
        }
    }

    public void score() {
        if (subState == SubStates.READY) {
            subState = SubStates.SCORE;
        }
    }

    /**
     * Go to amp position.
     * 
     * @param yn a boolean for whether to start or stop amp position (stopping stows by default).
     */
    public void amp(boolean yn) {
        if (yn) {
            if (subState != SubStates.SCORE && mainState != MainStates.AMP) { // If we're actively scoring, finish that score before continuing.
                mainState = MainStates.AMP;
                subState = SubStates.PREP;
            }
        }
        else {
            if (subState != SubStates.SCORE && mainState == MainStates.AMP) {
                mainState = MainStates.HOME;
                subState = SubStates.NOTHING;
            }
        }
    }

    public void speaker(boolean yn) {
        if (yn) {
            if (mainState != MainStates.SPEAKER && subState != SubStates.SCORE) {
                mainState = MainStates.SPEAKER;
                subState = SubStates.PREP;
            }
        } else {
            if (mainState == MainStates.SPEAKER && subState != SubStates.SCORE) {
                mainState = MainStates.HOME;
                subState = SubStates.NOTHING;
            }
        }
    }
}
