package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BunnyDropper {
    CANSparkMax dropperMotor;
    RelativeEncoder dropperEncoder;
    SparkMaxPIDController dropperController;

    public enum States {
        START, HOLD, DROP
    }

    States dropperStates;

    // BunnyDropper constructor, creates and initilializes hardware objects
    public BunnyDropper() {
        dropperMotor = new CANSparkMax(Constants.BUNNY_DROPPER_CANID, MotorType.kBrushless);
        dropperMotor.stopMotor();
        dropperEncoder = dropperMotor.getEncoder();
        dropperController = dropperMotor.getPIDController();
        dropperController.setP(Constants.BUNNY_DROPPER_P);
        dropperController.setI(Constants.BUNNY_DROPPER_I);
        dropperController.setD(Constants.BUNNY_DROPPER_D);
        dropperStates = States.START;

    }

    public boolean dropBunny(States state) {
        dropperStates = state;
        // dropping the bunny
        switch (dropperStates) {
            default:
            case START:
                dropperController.setReference(0, ControlType.kPosition);
                break;
            case HOLD:
                dropperController.setReference(Constants.BUNNY_DROPPER_HOLD_POSITION, ControlType.kPosition);
                break;
            case DROP:
                dropperController.setReference(Constants.BUNNY_DROPPER_DROP_POSITION, ControlType.kPosition);
                if (Math.abs(dropperEncoder.getPosition() - Constants.BUNNY_DROPPER_DROP_POSITION) < 0.1) {
                    return true;
                }
                break;
        }
        return false;
    }
// Testing if motors move. Speed should be scaled down value of left joystick y
    public void testPlanOne(double speed) {
        dropperMotor.set(speed);
    }

    // spinning motor at scaled joystick y value, records postition on smart
    // dashboard
    public void testPlanTwo(double speed) {
        dropperMotor.set(speed);
        SmartDashboard.putNumber("bunnydropper position",
                dropperEncoder.getPosition() * Constants.BUNNY_DROPPER_GEAR_RATIO);
    }

    // moving it to set position and when at said position, speed is set to 0
    public void testPlanThree() {
        dropperController.setReference(Constants.BUNNY_DROPPER_DROP_POSITION * Constants.BUNNY_DROPPER_GEAR_RATIO,
                ControlType.kPosition, 0);
    }

}