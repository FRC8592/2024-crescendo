package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
public class DriveTrain{

  //Create the motor objects
  private WPI_TalonFX frontLeft;
  private WPI_TalonFX backLeft;
  private WPI_TalonFX frontRight;
  private WPI_TalonFX backRight;
  private double targetRotation;
  private PIDController turnController = new PIDController(0.0033, 0, 0);
  
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

  public DriveTrain(){
    //define left side motor objects
    frontLeft = new WPI_TalonFX(Constants.LEFT_FRONT_CAN_ID);
    backLeft = new WPI_TalonFX(Constants.LEFT_BACK_CAN_ID);
    
    //define right side motor objects
    frontRight = new WPI_TalonFX(Constants.RIGHT_FRONT_CAN_ID);
    backRight = new WPI_TalonFX(Constants.RIGHT_BACK_CAN_ID);
    
    //configure frontLeft motor
    frontLeft.configFactoryDefault();
    frontLeft.setInverted(true);

    //configure backLeft motor
    backLeft.configFactoryDefault();
    backLeft.setInverted(true);
    backLeft.follow(frontLeft);

    //configure frontRight motor
    frontRight.configFactoryDefault();

    //configure backRight motor
    backRight.configFactoryDefault();
    backRight.follow(frontRight);
  }

  /*
  * ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
  *
  * UP: Motor creation, definitions, and config
  * DOWN: Drive train control functions
  *
  * ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
  */

  //Drive based on the left and right joystick y-positions (up-down in the real world)
  // public void tankDrive(double leftStick, double rightStick){
  //   frontLeft.set(ControlMode.PercentOutput, deadBand(leftStick * constants.motorPowerMultiplier));
  //   frontRight.set(ControlMode.PercentOutput, deadBand(rightStick * constants.motorPowerMultiplier));

  //   //Send motor speeds to SmartDashboard for debugging
  //   SmartDashboard.putNumber("Left Side", deadBand(leftStick * constants.motorPowerMultiplier));
  //   SmartDashboard.putNumber("Right Side", deadBand(rightStick * constants.motorPowerMultiplier));
  // }

  //Drive based on the X and Y coordinates of just one joystick
  public void xyDrive(double stickX, double stickY){
    
    //The Y (up-down) position of the stick is speed and the X position is rotation. Here's how we convert from that to 
    //motor speeds: Left motor is speed PLUS rotation. Right motor is speed MINUS rotation. If you think of 
    //rotating to the RIGHT as rotating in a POSITIVE direction, that means the LEFT motor speed must INCREASE and the 
    //RIGHT motor speed must DECREASE; hence, leftMotor is speed+rotation and rightMotor is speed-rotation.
    double leftMotor = stickY+stickX; 
    double rightMotor = stickY-stickX;
    leftMotor = leftMotor * Constants.motorPowerMultiplier;
    rightMotor = rightMotor * Constants.motorPowerMultiplier;

    double motorScale = 1; //This is another motor power multiplier, which we use to account for >100% motor speeds.
    

    //In some cases, the formula above can output numbers more than 1 on one of the tracks, so we set
    //our multiplier so that the motor that is more than 1 will run at exactly 1, and the other will be
    //scaled appropriately
    if(Math.abs(leftMotor)>1){
      motorScale = 1/Math.abs(leftMotor);
    }
    if(Math.abs(rightMotor)>1){
      motorScale = 1/Math.abs(rightMotor);
    }
    rightMotor = deadBand(rightMotor*motorScale);
    leftMotor = deadBand(leftMotor*motorScale);
    frontLeft.set(ControlMode.PercentOutput, rightMotor); //Now that we know that no motor will be greater than 1, it is
    frontRight.set(ControlMode.PercentOutput, leftMotor); //safe to assign the motors our calculated speeds

    //Once the motors are done with their thing, send their speeds to SmartDashboard for debugging
    SmartDashboard.putNumber("Left Side", leftMotor);
    SmartDashboard.putNumber("Right Side", rightMotor);
  }
  /*
   * This is relative to current rotation.
   */
  public void updateRotation(double offset){
    double turnSpeed = turnController.calculate(offset);
    if(turnSpeed<-1){turnSpeed=-1;} 
    if(turnSpeed>1){turnSpeed=1;}
    frontLeft.set(ControlMode.PercentOutput, turnSpeed);
    frontRight.set(ControlMode.PercentOutput, -turnSpeed);
  }
  //Stop the robot
  public void haltDriveTrain(){
    frontLeft.set(ControlMode.PercentOutput, 0);
    frontRight.set(ControlMode.PercentOutput, 0);

    //Not strictly necessary, but the numbers on SmartDashboard might read more than 0 if we don't do this
    SmartDashboard.putNumber("Left Side", 0);
    SmartDashboard.putNumber("Right Side", 0);
  }

  /*
  * ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
  *
  * UP: Drive train control functions
  * DOWN: Custom math functions
  *
  * ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
  */

  //Deadband method for the motors
  public double deadBand(double value){
    if(Math.abs(value) <= Constants.deadBandValue){
      return 0;
    }
    else{
      return value;
    }
  }
}