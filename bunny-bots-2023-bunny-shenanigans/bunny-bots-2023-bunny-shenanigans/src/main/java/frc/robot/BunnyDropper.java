package frc.robot;

import frc.robot.Constants;
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
    //BunnyDropper constructor, creates and initilializes hardware objects
    public BunnyDropper() {
        dropperMotor = new CANSparkMax(Constants.BUNNY_DROPPER_CANID, MotorType.kBrushless);
        dropperMotor.stopMotor();
        dropperEncoder = dropperMotor.getEncoder();
        dropperController  = dropperMotor.getPIDController();
        
    }

    public boolean dropBunny(){
        //dropping the bunny, when the motor reaches the max position the speed is set to zero
        dropperController.setReference(0.5* Constants.BUNNY_DROPPER_GEAR_RATIO, ControlType.kPosition, 0);
        if(dropperEncoder.getPosition()>= Constants.BUNNY_DROPPER_MAXIMUM_POSITION){
            dropperMotor.set(0);
            return true;
        }
        else{
            dropperMotor.set(Constants.BUNNY_DROPPER_MOTOR_SPEED*Constants.BUNNY_DROPPER_GEAR_RATIO);
            return false;
        }
    }
        //Testing if motors move. Speed should be scaled down value of left joystick y 
    public void testPlanOne(double speed){
        dropperMotor.set(speed);
    }
    
    public void testPlanTwo(double speed){
        dropperMotor.set(speed);
        SmartDashboard.putNumber("bunnydropper position", dropperEncoder.getPosition()*Constants.BUNNY_DROPPER_GEAR_RATIO);
    }
    
    public void testPlanThree(){
        
        if(dropperEncoder.getPosition()>= Constants.BUNNY_DROPPER_MAXIMUM_POSITION){
            dropperMotor.set(0);
        }
        else{
            dropperMotor.set(Constants.BUNNY_DROPPER_MOTOR_SPEED*Constants.BUNNY_DROPPER_GEAR_RATIO);
        }
    }

}

