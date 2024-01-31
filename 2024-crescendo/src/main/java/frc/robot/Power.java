//////////////////////////////////////////////////////////////////////////////////////////////////////
// Power measurement and control
/////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Power {
    //Constants
    private static final int VOLTAGE_SMOOTHING_LENGTH = 50;
    private static final double DISABLED_LOW_BATTERY_VOLTAGE = 11.5;
    private static final double TELEOP_LOW_BATTERY_VOLTAGE = 10.5;
    private static final String LOG_PATH = "CustomLogs/Power/";
    
    // Low battery warning
    private double currentBatteryVoltage;
    private ArrayList<Double> voltages;
    private boolean isBatteryLow = false;
    
    // Object variables
    private PowerDistribution revPDH;

    //
    // Shared power variables
    //
    public double temp;
    public double voltage;
    public double current;
    public double power;
    public double energy;

    // Shuffleboard for power data
    ShuffleboardTab powerTab;


    //
    // Constructor for power control
    //
    public Power() {
        // Create new Rev Power Distribution object
       revPDH = new PowerDistribution(Constants.PDH_CAN, PowerDistribution.ModuleType.kRev);
        
        // Create the shuffleboard tab for power data
        powerTab = Shuffleboard.getTab("Power");
        
        voltages = new ArrayList<Double>();

    }


    //
    // Periodically post power data to the dashboard
    //
    public void powerPeriodic() {
        current = revPDH.getTotalCurrent();
        // Get parameters from the PDH
        
        temp    = revPDH.getTemperature();
        voltage = revPDH.getVoltage();
        power   = revPDH.getTotalPower();
        energy  = revPDH.getTotalEnergy();
        
        voltages.add(0, RobotController.getBatteryVoltage());
        if (voltages.size() > VOLTAGE_SMOOTHING_LENGTH) {
            voltages.remove(VOLTAGE_SMOOTHING_LENGTH);
        }
        double x = 0.0;
        for (double i : voltages) {
            x += i;
        }
        currentBatteryVoltage = x / VOLTAGE_SMOOTHING_LENGTH;
        if (DriverStation.isDisabled() || DriverStation.isAutonomous() || DriverStation.isTest()) {
            if (currentBatteryVoltage < DISABLED_LOW_BATTERY_VOLTAGE) {
                isBatteryLow = true;
            }
        }

        if (DriverStation.isTeleop()) {
            if (currentBatteryVoltage < TELEOP_LOW_BATTERY_VOLTAGE) {
                isBatteryLow = true;
            }
        }

        Logger.recordOutput(LOG_PATH + "Is Battery Low", isBatteryLow);
        SmartDashboard.putBoolean("Is Battery Low", isBatteryLow);
        // Place all parameters onto a dedicated Shuffleboard tab
        // Shuffleboard.selectTab("Power");

        // SmartDashboard.putNumber("Temperature", temp);
        // SmartDashboard.putNumber("Voltage", voltage);
        // SmartDashboard.putNumber("Current", current);
        // SmartDashboard.putNumber("Voltage", voltage);
        // SmartDashboard.putNumber("Current", current);
        // SmartDashboard.putNumber("Power", power);
        // SmartDashboard.putNumber("Energy", energy);

        // Shuffleboard.selectTab("SmartDashboard");   // Switch back to default
    }


    //
    // Turn the switchable 12v port on
    //
    public void relayOn() {
        // revPDH.setSwitchableChannel(true);
    }


    //
    // Turn the switchable 12v port off
    //
    public void relayOff() {
        // revPDH.setSwitchableChannel(false);
    }
}