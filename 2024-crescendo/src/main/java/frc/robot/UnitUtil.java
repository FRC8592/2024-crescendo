package frc.robot;

import frc.robot.Constants.*;

public class UnitUtil {
    public static double metersPerSecondToTicks(double input){
        return input * CONVERSIONS.METERS_SECOND_TO_TICKS_TALONFX;
    }

    public static double ticksToMetersPerSecond(double input) {
        return input / CONVERSIONS.METERS_SECOND_TO_TICKS_TALONFX;
    }

    public static double ticksToRPM(double input) {
        return input / CONVERSIONS.RPM_TO_TICKS_100_MS_TALONFX;
    }

    public static double RPMToTicks(double input) {
        return input * CONVERSIONS.RPM_TO_TICKS_100_MS_TALONFX;
    }
}
