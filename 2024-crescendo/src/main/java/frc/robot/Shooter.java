package frc.robot;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.*;
//import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.SparkPIDController;

public class Shooter {
    
    // private final String shooterTableName = "shooter_table"; TODO: What are these three lines?
    // private final NetworkTable table;
    // private NetworkTableEntry shooterSpeedRPS;

    public SparkFlexControl topShooterMotor;
    public SparkFlexControl bottomShooterMotor;
    // SparkPIDController leftShooterControl;
    // SparkPIDController rightShooterControl;
    public DigitalInput noteBeamBreak; // beam break sensor top/bottom mounted

    public SparkFlexControl feederMotor;

    int toptargetSpeed = 0;
    int bottomTargetSpeed = 0;

    public boolean hasNote;

    public enum IntakeStates{
        INTAKING,
        REPOSITION,
        NOTHING
    }

    private Timer movebackTimer;

    public IntakeStates state;

    /**
     * Shooter object constructor
     */
    public Shooter() {
        topShooterMotor = new SparkFlexControl(CAN.TOP_SHOOTER_MOTOR_CAN_ID, false);
        topShooterMotor.setInverted();
        bottomShooterMotor = new SparkFlexControl(CAN.BOTTOM_SHOOTER_MOTOR_CAN_ID, false);
        feederMotor = new SparkFlexControl(CAN.FEEDER_MOTOR_CAN_ID, false);

        // table = NetworkTableInstance.getDefault().getTable(shooterTableName);
        topShooterMotor.setPIDF(SHOOTER.TOP_SHOOTER_MOTOR_kP, SHOOTER.TOP_SHOOTER_MOTOR_kI, SHOOTER.TOP_SHOOTER_MOTOR_kD, SHOOTER.TOP_SHOOTER_MOTOR_kF, 0);
        bottomShooterMotor.setPIDF(SHOOTER.BOTTOM_SHOOTER_MOTOR_kP, SHOOTER.BOTTOM_SHOOTER_MOTOR_kI, SHOOTER.BOTTOM_SHOOTER_MOTOR_kD, SHOOTER.BOTTOM_SHOOTER_MOTOR_kF, 0);

        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_kP, SHOOTER.FEEDER_MOTOR_kI, SHOOTER.FEEDER_MOTOR_kD, SHOOTER.FEEDER_MOTOR_kF, 0);
        feederMotor.setInverted();

        bottomShooterMotor.setPercentOutput(0);
        topShooterMotor.setPercentOutput(0);
        feederMotor.setPercentOutput(0);

        noteBeamBreak = new DigitalInput(SHOOTER.NOTE_BEAM_BREAK_DIO_PORT);

        bottomShooterMotor.follow(topShooterMotor, true);

        topShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);
        bottomShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);

        movebackTimer = new Timer();
    }

    public void intakeUpdate(){ // We only put a state machine on intaking
        switch (state) {
            case NOTHING:
            default:
                break;
            case INTAKING:
                setShootVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED, SHOOTER.INTAKE_FLYWHEEL_SPEED);
                setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED);
                if(hasNote()){ //TODO: Note that this depends on hasNote working properly. It should on the new shooter, but it will be prone to random issues at the time of writing this code
                    state = IntakeStates.REPOSITION;
                    movebackTimer.start();
                }
                break;
            case REPOSITION:
                setShootVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED, SHOOTER.INTAKE_FLYWHEEL_SPEED);
                if(!movebackTimer.hasElapsed(SHOOTER.REPOSITION_TIME)){ //Note the ! at the start of the condition
                    setFeederVelocity(SHOOTER.REPOSITION_SPEED);
                }
                else{
                    setFeederVelocity(0);
                    state = IntakeStates.NOTHING;
                }
                break;
        }
    }

    /**
     * set shooter motors speeds in terms of RPM
     * @param speedRPM
     */
    public void setShootVelocity(int topspeedRPM, int bottomspeedRPM){
        state = IntakeStates.NOTHING; // Stop whatever intake thing we're doing to avoid sending the motor multiple commands at the same time
        topShooterMotor.setVelocity(topspeedRPM);
        bottomShooterMotor.setVelocity(bottomspeedRPM);
        toptargetSpeed = topspeedRPM;
        bottomTargetSpeed = bottomspeedRPM;
    }

    /**
     * set shooter motors speeds in terms of percent output
     * @param power
     */
    public void setShootPower(double power){
        state = IntakeStates.NOTHING;
        topShooterMotor.setPercentOutput(power);
        bottomShooterMotor.setPercentOutput(power);
    }

    /**
     * Stops the flywheels
     */
    public void stop() {
        state = IntakeStates.NOTHING;
        topShooterMotor.stop();
        bottomShooterMotor.stop();
    }

    /**
     * Sets speed of the feeder wheels in terms of percent output
     * @param power
     */
    public void setFeederPower(double power) {
        state = IntakeStates.NOTHING;
        feederMotor.setPercentOutput(power);
    }

    /**
     * Sets velocity of the feeder wheels
     * @param feederVelocity feeder velocity in RPM
    */
    public void setFeederVelocity(double feederVelocity) {
        state = IntakeStates.NOTHING;
        feederMotor.setVelocity(feederVelocity);
    }

    /**
     * stops feeder wheels from moving
     */
    public void stopFeeders() {
        state = IntakeStates.NOTHING;
        feederMotor.stop();
    }

    /**
     * Checks whether note is in shooter
     * @return hasNote (boolean)
     */
    public boolean hasNote() {
        boolean currentlyHasNote = !noteBeamBreak.get(); // true if beam is BROKEN! thus note beam is not broken, no note
        if (currentlyHasNote) {
            hasNote = true;
        }
        return hasNote;
    }

    // /**
    //  * sets speed and angle based on range table
    //  * @param distanceToAprilTag
    //  */
    // public void setSpeedRangeTable(double distanceToAprilTag, Elevator elevator) {
    //     int index = (int)(distanceToAprilTag / CONVERSIONS.METERS_TO_FEET);
    //     double[] vals = SHOOTER.RANGE_TABLE[index];
    //     double angle = vals[0];
    //     int targetSpeed = (int)vals[1];
    //     // setShootVelocity(targetSpeed);/
    //     elevator.setPivotAngleCustom(angle);
    // }

    /**
     * checks if flywheels are at target speed to shoot
     * and resets hasNote cuz that means we're gonna shoot
     * 
     * @apiNote You must call this method before shooting
     */
    public boolean isReady() {
        SmartDashboard.putNumber("topShooterRPM", topShooterMotor.getVelocity());
        SmartDashboard.putNumber("bottomShooterRPM", bottomShooterMotor.getVelocity());
        if (Math.abs(topShooterMotor.getVelocity() - toptargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE&&
                Math.abs(bottomShooterMotor.getVelocity() - bottomTargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE) {
            hasNote = false;
            return true;
        }
        return false;
    }

    /**
     * Starts the intake state machine; run the feeders until we have a note, then back it off a little. All methods other than {@code intake()}, {@code hasNote()}, {@code isReady()}, and {@code intakeUpdate()} will cancel whatever the state machine is doing and prioritize that method's function.
     */
    public void intake(){
        this.state = IntakeStates.INTAKING;
    }
}