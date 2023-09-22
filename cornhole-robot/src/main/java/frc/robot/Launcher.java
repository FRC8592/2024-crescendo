// ---------------------------------------------------------------------------
// Cornhole robot launcher control
// ---------------------------------------------------------------------------

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.wpilog.*;
import org.littletonrobotics.junction.networktables.*;
import frc.robot.Constants;


public class Launcher {

    //
    // Object variables for motors
    //
    private WPI_TalonFX launchMotor1;
    private WPI_TalonFX launchMotor2;
    private double posLastFrame;//This will hold the position of the motor the frame before the current frame
    private enum States {IDLE, IDLE_WAIT, LAUNCH, DOWN, UP};
    private States state;
    private Timer timer;

    //
    // Create all Launcher motor instances
    //
    public Launcher() {
        // Define the launch motor
        launchMotor1 = new WPI_TalonFX(Constants.LAUNCH_CAN_ID);

        // Configure the launch motor
        launchMotor1.configFactoryDefault();
        launchMotor1.setNeutralMode(NeutralMode.Brake);
        launchMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        launchMotor1.setSelectedSensorPosition(0);
        launchMotor1.config_kP(0,Constants.LAUNCH_MOTOR_kP);
        launchMotor1.config_kI(0,Constants.LAUNCH_MOTOR_kI);
        launchMotor1.config_kD(0,Constants.LAUNCH_MOTOR_kD);
        launchMotor1.config_kF(0,Constants.LAUNCH_MOTOR_kF);
        launchMotor1.config_kP(1,Constants.LAUNCH_MOTOR_kP_BRAKE);
        launchMotor1.config_kI(1,Constants.LAUNCH_MOTOR_kI_BRAKE);
        launchMotor1.config_kD(1,Constants.LAUNCH_MOTOR_kD_BRAKE);
        launchMotor1.config_kF(1,Constants.LAUNCH_MOTOR_kF_BRAKE);
        launchMotor1.selectProfileSlot(0, 0);
        launchMotor2 = new WPI_TalonFX(Constants.LAUNCH_2_CAN_ID);

        // Configure the launch motor
        launchMotor2.configFactoryDefault();
        launchMotor2.setNeutralMode(NeutralMode.Brake);
        launchMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        launchMotor2.setSelectedSensorPosition(0);
        launchMotor2.setInverted(true);
        launchMotor2.follow(launchMotor1);
        posLastFrame=0;
        state = States.IDLE;
        timer = new Timer();
    }
    public void update(){ //Should be called constantly in teleopPeriodic()
        double localPos=launchMotor1.getSelectedSensorPosition();
        Logger.getInstance().recordOutput("Launcher/Position", localPos);
        Logger.getInstance().recordOutput("Launcher/MovementPerFrame", localPos-posLastFrame);
        Logger.getInstance().recordOutput("Launcher/LauncherSpeed", launchMotor1.getSelectedSensorVelocity());
        Logger.getInstance().recordOutput("Launcher/State", state.name());
        posLastFrame=localPos;
        switch(state){ //This state machine is shown in chronological order.

            case UP:
                if(launchMotor1.getSelectedSensorPosition()<1000){
                    launchMotor1.set(ControlMode.PercentOutput, 0.12);
                }
                else{
                    launchMotor1.set(ControlMode.PercentOutput, 0.07);
                }
                break;

            case LAUNCH: //Activated from a button combo in Robot.java. 
                launchMotor1.selectProfileSlot(0, 0);
                SmartDashboard.putNumber("CATAPAULT ENCODER POS", localPos);
                Logger.getInstance().recordOutput("Launcher/IsLaunching", true);

                if (localPos >= Constants.LAUNCH_MAX_POSITION) { //If it's time to stop
                    Logger.getInstance().recordOutput("Launcher/PositionGreaterThanMax", true);
                    timer.reset(); // These two lines are prep for
                    timer.start(); // IDLE_WAIT's timeout function.
                    state=States.IDLE_WAIT;
                    
                    //More immediate response time
                    launchMotor1.selectProfileSlot(1, 0);
                    launchMotor1.set(ControlMode.Velocity, 0);
                    Logger.getInstance().recordOutput("Launcher/IsLaunching", false);
                }
                else{
                    Logger.getInstance().recordOutput("Launcher/PositionGreaterThanMax", false);
                    //launchMotor1.set(ControlMode.Velocity, Constants.LAUNCH_VELOCTY_TICKS_100MS);
                    launchMotor1.set(ControlMode.PercentOutput, 1.0);
                }
                break;

            case IDLE_WAIT: // Does nothing for 2 seconds, then switches the state to lowering the arm
                launchMotor1.selectProfileSlot(1, 0);
                launchMotor1.set(ControlMode.Velocity, 0);
                Logger.getInstance().recordOutput("Launcher/IsLaunching", false);
                if(timer.get()>0.5){
                    state=States.DOWN;
                }
                break;

            case DOWN: // Lowers the arm to the ground in preparation for the next throw.
                launchMotor1.set(ControlMode.PercentOutput, Constants.LAUNCH_DOWN_POWER);
                Logger.getInstance().recordOutput("Launcher/IsLaunching", false);

                if ( localPos <= Constants.LAUNCH_MAX_POSITION) { //For logging
                    Logger.getInstance().recordOutput("Launcher/PositionGreaterThanMax", false);
                }

                if ( localPos <= Constants.DOWN_MIN_POSITION) { 
                    state=States.IDLE;
                }
                break;

            case IDLE:
            default: //Doing nothing
                launchMotor1.set(ControlMode.PercentOutput, 0.0);
                Logger.getInstance().recordOutput("Launcher/IsLaunching", false);
                break;
        }
    }
    // //
    // // Get the encoder value of the launch motor
    // //
    // public boolean launchCheckMaxPosition() {
    //     double localPos;

    //     localPos = launchMotor1.getSelectedSensorPosition();

    //     SmartDashboard.putNumber("CATAPAULT ENCODER POS", localPos);

    //     if ( localPos >= Constants.LAUNCH_MAX_POSITION) {
    //         launchMotor1.set(ControlMode.PercentOutput, 0.0);
    //         Logger.getInstance().recordOutput("Launcher/PositionGreaterThanMax", true);
    //         return true;
    //     }
    //     Logger.getInstance().recordOutput("Launcher/PositionGreaterThanMax", false);
    //     return false;
    // }


    //
    // Launch the beanbag
    //
    // For now this is a simple motor power controller
    //
    public void launch() {
        if(state == States.IDLE){
            launchMotor2.setSelectedSensorPosition(0);
            launchMotor1.setSelectedSensorPosition(0);
            state=States.LAUNCH;
        }
    }
    public void setIdle(){
        state=States.IDLE;
    }
    public void up(){
        state=States.UP;
    }


    // //
    // // Brake (reverse) the beanbag launch mechanism
    // //
    // // For now this is a simple motor power controller
    // //
    // public void launchDown() {
    //     launchMotor1.set(ControlMode.PercentOutput, Constants.LAUNCH_BRAKE_POWER);
    //     Logger.getInstance().recordOutput("Launcher/IsLaunching", false);
    // }

    // //
    // // Idle the beanbag launch mechanism
    // //
    // // For now this is a simple motor power controller
    // //
    // public void launchIdle() {
    //     launchMotor1.set(ControlMode.PercentOutput, 0.0);
    //     Logger.getInstance().recordOutput("Launcher/IsLaunching", false);
    // }
    public void reZero(){
        launchMotor2.setSelectedSensorPosition(0);
        launchMotor1.setSelectedSensorPosition(0);
    }

}