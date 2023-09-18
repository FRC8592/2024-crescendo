// ---------------------------------------------------------------------------
// Cornhole robot launcher control
// ---------------------------------------------------------------------------

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class Launcher {

    // Values and variables for the launcher state machine
    private static enum launcherStates {LAUNCHER_IDLE, LAUNCHER_LAUNCH, LAUNCHER_RESET}
    private launcherStates launcherState;

    // Launch readiness and command
    private boolean launchReady  = false;
    private boolean launchActive = false;

    // Object variables for motors
    private WPI_TalonFX launchMotor;


    // Create all Launcher motor instances
    public Launcher() {

        // The launch arm MUST start in the fully reset condition
        launcherState = launcherStates.LAUNCHER_IDLE;

        // Define the launch motor
        launchMotor = new WPI_TalonFX(Constants.LAUNCH_CAN_ID);

        // Basic configuration for the launch motor
        launchMotor.configFactoryDefault();
        launchMotor.setNeutralMode(NeutralMode.Coast);

        // Configure voltage compensation to help maintain stable speed as battery voltage changes
        launchMotor.configVoltageCompSaturation(Constants.LAUNCH_VOLTAGE);
        launchMotor.enableVoltageCompensation(true);   // Enable voltage compensation

        // Settings for launch arm PID constant velocity mode
        launchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        launchMotor.setSelectedSensorPosition(0);   // Reset the sensor.  It must always start in the parked position
     
        launchMotor.config_kP(0, Constants.LAUNCH_P);
        launchMotor.config_kI(0, Constants.LAUNCH_I);
        launchMotor.config_kD(0, Constants.LAUNCH_D);
        launchMotor.config_kF(0, Constants.LAUNCH_F);

        // Set for velocity mode using the constants in slot 0
        launchMotor.selectProfileSlot(0, 0);
        launchMotor.set(ControlMode.Velocity, 0);

        // Provide a ramp rate for 0 to full power
        launchMotor.configClosedloopRamp(1);
    }


    //
    // Set the flag to indicate the launch sequence should commence
    //
    public void setLaunch() {
        launchActive = true;
    }


    //
    // clear the flag to indicate that the launcher should sit idle
    //
    public void clearLaunch() {
        launchActive = false;
    }


    //
    // Remove motor power and set the motor to Coast mode
    //
    public void launchIdle() {
        launchMotor.set(ControlMode.PercentOutput, 0.0);
        launchMotor.setNeutralMode(NeutralMode.Coast);    
    }


    /**
     * Execute launch sequence and reset
     * @return
     */
    public void launchControl() {
        double launchPosition;      // Launch motor encoder value

        // Read motor encoder value
        launchPosition = launchMotor.getSelectedSensorPosition();

        // Push values to the Smart Dashboard
        SmartDashboard.putBoolean("Launch Ready", launchReady);
        SmartDashboard.putNumber("Launch Position", launchPosition);

        //
        // State machine to control the launch sequence
        //
        switch(launcherState) {

            // Sit at idle until the launch command arrives
            case LAUNCHER_IDLE:
                launchMotor.set(ControlMode.PercentOutput, 0.0);
                launchMotor.setNeutralMode(NeutralMode.Coast);
                launchReady = true;

                if (launchActive) {
                    launchReady   = false;
                    launcherState = launcherStates.LAUNCHER_LAUNCH;
                }


            // Launch the beanbag
            case LAUNCHER_LAUNCH:
                launchMotor.set(ControlMode.Velocity, Constants.LAUNCH_RPM);    // Closed loop velocity control
        
                // Stop the arm when we reach our launch position
                if (launchPosition >= Constants.LAUNCH_MAX_POSITION) {
                    launchMotor.setNeutralMode(NeutralMode.Brake);             // Activate motor brake
                    launcherState = launcherStates.LAUNCHER_RESET;
                }


            // Reset the launch ARM
            case LAUNCHER_RESET:
                launchMotor.set(ControlMode.PercentOutput, Constants.LAUNCH_RESET_POWER);    // Slowly return the arm to the parked position

                if (launchPosition <= 100.0) {
                    clearLaunch();
                    launcherState = launcherStates.LAUNCHER_IDLE;
                }

        }
    }
}