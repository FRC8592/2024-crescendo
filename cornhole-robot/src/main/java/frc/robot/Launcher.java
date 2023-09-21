// ---------------------------------------------------------------------------
// Cornhole robot launcher control
// ---------------------------------------------------------------------------

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class Launcher {
    private double launchPosition;
    private double launchVelocity;

    //
    // Object variables for motors
    //
    private WPI_TalonFX launchMotor;
    private WPI_TalonFX launchMotor2;


    //
    // Create all Launcher motor instances
    //
    public Launcher() {
        // Define the launch motor
        launchMotor = new WPI_TalonFX(Constants.LAUNCH_CAN_ID);
        launchMotor2 = new WPI_TalonFX(Constants.LAUNCH_CAN_ID2);

        // Configure the launch motor
        launchMotor.configFactoryDefault();
        launchMotor.setNeutralMode(NeutralMode.Brake);
        launchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        launchMotor.setSelectedSensorPosition(0);
        launchMotor.configOpenloopRamp(0.1, 0);

        launchMotor2.configFactoryDefault();
        launchMotor2.follow(launchMotor);
        launchMotor2.setInverted(InvertType.OpposeMaster);
        launchMotor2.setNeutralMode(NeutralMode.Brake);
        launchMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        launchMotor2.setSelectedSensorPosition(0);
        launchMotor2.configOpenloopRamp(0.1, 0);

    }


    //
    // Execute periodic functions.  In this case, we're mostly gathering data
    //
    public void launchPeriodic() {
        launchPosition = launchMotor.getSelectedSensorPosition();
        launchVelocity = launchMotor.getSelectedSensorVelocity();

        SmartDashboard.putNumber("Launch Position", launchPosition);
        SmartDashboard.putNumber("Launch Velocity", launchVelocity);
    }


    //
    // Get the encoder value of the launch motor
    //
    public boolean launchCheckMaxPosition() {

        if ( launchPosition >= Constants.LAUNCH_MAX_POSITION) {
            launchMotor.set(ControlMode.PercentOutput, 0.0);
            return true;
        }
        return false;
    }


    //
    // Launch the beanbag
    //
    // For now this is a simple motor power controller
    //
    public void launchAccel() {
        launchMotor.set(ControlMode.PercentOutput, Constants.LAUNCH_ACCEL_POWER);
    }


    //
    // Brake (reverse) the beanbag launch mechanism
    //
    // For now this is a simple motor power controller
    //
    public void launchDown() {
        launchMotor.set(ControlMode.PercentOutput, Constants.LAUNCH_BRAKE_POWER);
    }

    //
    // Idle the beanbag launch mechanism
    //
    // For now this is a simple motor power controller
    //
    public void launchIdle() {
        launchMotor.set(ControlMode.PercentOutput, 0.0);
    }

}