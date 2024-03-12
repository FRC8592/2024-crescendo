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

import frc.robot.Constants.*;


public class Power {
    //Constants
    
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
       revPDH = new PowerDistribution(CAN.PDH_CAN_ID, PowerDistribution.ModuleType.kRev);
        
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

        voltages.add(0, voltage);
        if (voltages.size() > POWER.VOLTAGE_SMOOTHING_LENGTH) {
            voltages.remove(POWER.VOLTAGE_SMOOTHING_LENGTH);
        }
        double x = 0.0;
        for (double i : voltages) {
            x += i;
        }
        currentBatteryVoltage = x / POWER.VOLTAGE_SMOOTHING_LENGTH;
        if (DriverStation.isDisabled() || DriverStation.isAutonomous() || DriverStation.isTest()) {
            if (currentBatteryVoltage < POWER.DISABLED_LOW_BATTERY_VOLTAGE) {
                isBatteryLow = true;
            }
        }

        if (DriverStation.isTeleop()) {
            if (currentBatteryVoltage < POWER.TELEOP_LOW_BATTERY_VOLTAGE) {
                isBatteryLow = true;
            }
        }

        Logger.recordOutput(POWER.LOG_PATH + "Is Battery Low", isBatteryLow);
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