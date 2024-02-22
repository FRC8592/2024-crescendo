package frc.robot;

import frc.robot.Constants.*;

public class UnitUtil {
    public static double metersPerSecondToTicks(double input){
        return input * CONVERSIONS.METERS_SECOND_TO_TICKS;
    }

    public static double ticksToMetersPerSecond(double input) {
        return input / CONVERSIONS.METERS_SECOND_TO_TICKS;
    }

    public static double ticksToRPM(double input) {
        return input / CONVERSIONS.RPM_TO_TICKS_100_MS;
    }

    public static double RPMToTicks(double input) {
        return input * CONVERSIONS.RPM_TO_TICKS_100_MS;
    }
}
