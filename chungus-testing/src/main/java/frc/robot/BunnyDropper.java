package frc.robot;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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
        dropperController.setP(Constants.BUNNY_DROPPER_kP);
        dropperController.setI(Constants.BUNNY_DROPPER_kI);
        dropperController.setD(Constants.BUNNY_DROPPER_kD);
        dropperStates = States.START;

    }

    public boolean dropBunny(States state) {
        dropperStates = state;
        // dropping the bunny
        Logger.getInstance().recordOutput("Bunny Dropper/Position", dropperEncoder.getPosition());
        switch (dropperStates) {
            default:
            case START:
                dropperController.setReference(0, ControlType.kPosition);
                Logger.getInstance().recordOutput("BunnyDropper/State", "START");
                break;
            case HOLD:
                dropperController.setReference(Constants.BUNNY_DROPPER_HOLD_POSITION, ControlType.kPosition);
                Logger.getInstance().recordOutput("BunnyDropper/State", "HOLD");
                break;
            case DROP:
                Logger.getInstance().recordOutput("BunnyDropper/State", "DROP");
                dropperController.setReference(Constants.BUNNY_DROPPER_DROP_POSITION, ControlType.kPosition);
                if (Math.abs(dropperEncoder.getPosition() - Constants.BUNNY_DROPPER_DROP_POSITION) > 0.1) {
                    return false;
                }
                break;
        }
        return true;
    }
// Testing if motors move. Speed should be scaled down value of left joystick y
    public void spinBunnyDropperPct(double speed) {
        dropperMotor.set(speed);
    }

    public void zero() {
        dropperEncoder.setPosition(0);
    }
    // // spinning motor at scaled joystick y value, records postition on smart
    // // dashboard
    // public void testPlanTwo(double speed) {
    //     dropperMotor.set(speed);
    //     SmartDashboard.putNumber("bunnydropper position",
    //             dropperEncoder.getPosition() * Constants.BUNNY_DROPPER_GEAR_RATIO);
    // }

    // // moving it to set position and when at said position, speed is set to 0
    // public void testPlanThree() {
    //     dropperController.setReference(Constants.BUNNY_DROPPER_DROP_POSITION * Constants.BUNNY_DROPPER_GEAR_RATIO,
    //             ControlType.kPosition, 0);
    // }

}