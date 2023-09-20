// ---------------------------------------------------------------------------
// Cornhole robot launcher control
// ---------------------------------------------------------------------------

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
        
        launchMotor2 = new WPI_TalonFX(Constants.LAUNCH_CAN_ID);

        // Configure the launch motor
        launchMotor2.configFactoryDefault();
        launchMotor2.setNeutralMode(NeutralMode.Brake);
        launchMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        launchMotor2.setSelectedSensorPosition(0);
        launchMotor2.setInverted(true);
        launchMotor2.follow(launchMotor1);
        posLastFrame=0;

    }
    public void updateLogging(){ //Should be called constantly in teleopPeriodic()
        double localPos=launchMotor1.getSelectedSensorPosition();
        Logger.getInstance().recordOutput("Launcher/Position", localPos);
        Logger.getInstance().recordOutput("Launcher/MovementPerFrame", localPos-posLastFrame);
        Logger.getInstance().recordOutput("Launcher/LauncherSpeed", launchMotor1.getSelectedSensorVelocity());
        posLastFrame=localPos;
    }
    //
    // Get the encoder value of the launch motor
    //
    public boolean launchCheckMaxPosition() {
        double localPos;

        localPos = launchMotor1.getSelectedSensorPosition();

        SmartDashboard.putNumber("CATAPAULT ENCODER POS", localPos);

        if ( localPos >= Constants.LAUNCH_MAX_POSITION) {
            launchMotor1.set(ControlMode.PercentOutput, 0.0);
            Logger.getInstance().recordOutput("Launcher/PositionGreaterThanMax", true);
            return true;
        }
        Logger.getInstance().recordOutput("Launcher/PositionGreaterThanMax", false);
        return false;
    }


    //
    // Launch the beanbag
    //
    // For now this is a simple motor power controller
    //
    public void launchAccel() {
        launchMotor1.set(ControlMode.PercentOutput, Constants.LAUNCH_ACCEL_POWER);
        Logger.getInstance().recordOutput("Launcher/IsLaunching", true);
    }


    //
    // Brake (reverse) the beanbag launch mechanism
    //
    // For now this is a simple motor power controller
    //
    public void launchDown() {
        launchMotor1.set(ControlMode.PercentOutput, Constants.LAUNCH_BRAKE_POWER);
        Logger.getInstance().recordOutput("Launcher/IsLaunching", false);
    }

    //
    // Idle the beanbag launch mechanism
    //
    // For now this is a simple motor power controller
    //
    public void launchIdle() {
        launchMotor1.set(ControlMode.PercentOutput, 0.0);
        Logger.getInstance().recordOutput("Launcher/IsLaunching", false);
    }

}