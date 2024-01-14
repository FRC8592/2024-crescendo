package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode; 

// Pneumatic control classes
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import packages
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    // declare roller and four-bar motors
    private WPI_TalonFX rollerMotor;

    // Pneumatics
    // private Compressor robotCompressor;
    // private DoubleSolenoid collectorSolenoid;

    int toggleCounter = 0; // Counter to toggle intake

    private enum States {
        EXTEND,
        RETRACT
    }

    private States state;
    public Intake() {
        state = States.RETRACT;
        rollerMotor = new WPI_TalonFX(Constants.INTAKE_ROLLER_MOTOR_CAN_ID);

        // Start Pneumatics
        // robotCompressor = new Compressor(PneumaticsModuleType.REVPH);

        // collectorSolenoid = new DoubleSolenoid(Constants.INTAKE_PNEUMATICS_CAN_ID,
        //        PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_PORT_A, Constants.INTAKE_SOLENOID_PORT_B);

        // *** DANGER : Closed loop control must be enabled to prevent overpressure ***
        // robotCompressor.enableDigital();
        // robotCompressor.enableAnalog(100, 120);; // Cycle to control pressure robotCompressor.start(); // Start compressor running
        // collectorSolenoid.set(DoubleSolenoid.Value.kForward); // Move collector inboard (undeploy it)

        //set motors to factory default
        rollerMotor.configFactoryDefault();

        //set motors to neutral mdode
        rollerMotor.setNeutralMode(NeutralMode.Brake);

        rollerMotor.config_kP(0, Constants.INTAKE_ROLLER_MOTOR_kP);
        rollerMotor.config_kI(0, 0);
        rollerMotor.config_kD(0, 0);

        rollerMotor.selectProfileSlot(0, 0);



    }

    public void update(boolean shootMode) {
        switch (state) {
            default:
            case RETRACT:
                if (!shootMode) {
                    rollerMotor.set(ControlMode.PercentOutput, 0);
                }
                // collectorSolenoid.set(DoubleSolenoid.Value.kForward);
                Logger.getInstance().recordOutput("Intake/SolenoidPosition", "kForward"); //Forward and reverse are inverted from how you would expect them to behave.
                break;
            case EXTEND:
                // collectorSolenoid.set(DoubleSolenoid.Value.kReverse);
                rollerMotor.set(ControlMode.PercentOutput, Constants.INTAKE_ROLLER_SPEED);
                Logger.getInstance().recordOutput("Intake/SolenoidPosition", "kReverse");
                break;
        }
        Logger.getInstance().recordOutput("Intake/RollerMotorSpeed (Ticks per 100ms)", rollerMotor.getSelectedSensorVelocity());
    }

    public void retractIntake(){
        state = States.RETRACT;
    }

    public void extendIntake(){
        state = States.EXTEND;
    }

    

    public void toggleRollerPosition() {
        if (toggleCounter % 2 == 0) {
            retractIntake();
            toggleCounter++;
        }
        else {
            extendIntake();
            toggleCounter++;
        }
    }


    /**
     * Clamps a value between a minimum and maximum value
     * 
     * @param value The value to clamp
     * @param min   The minimum value
     * @param max   The maximum value
     */
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }


    public void spinRoller(double velocity) {
        // spins the roller at a given power
        rollerMotor.set(ControlMode.Velocity, clamp(velocity, -1, 1));
        // Add values to Smart Dashboard for testing
        SmartDashboard.putNumber("Input Velocity of Roller Motor", velocity);
        SmartDashboard.putNumber("Actual velocity of Roller Motor", rollerMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Actual applied power of Roller Motor", rollerMotor.getMotorOutputPercent());


    }
    // This method converts velocity from m/s to ticks/s
    public static double metersPerSecondToTicksPer100ms(double input) {
        double output = ((input / 10) * (Constants.INTAKE_REVOLUTIONS_PER_METER) * Constants.MEASUREMENT_TICKS_PER_REVOLUTION) / Constants.INTAKE_GEAR_RATIO;
        return output;
    }

    public void spinRollerPID(double speedMPS) {
        double ticksPer100MS = metersPerSecondToTicksPer100ms(speedMPS);
        rollerMotor.set(ControlMode.Velocity, ticksPer100MS * Constants.INTAKE_ADJUST_ROLLER_TICKS_SCALE);
        SmartDashboard.putNumber("Roller ticks per 100 MS", ticksPer100MS);
    }

    public void spinIntakePercentOutput(double speed){
        rollerMotor.set(ControlMode.PercentOutput, speed);
    }



    public void TEST_extendIntake(){
        // collectorSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void TEST_retractIntake(){
        // collectorSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
