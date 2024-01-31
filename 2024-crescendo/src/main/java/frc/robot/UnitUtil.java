package frc.robot;

public class UnitUtil {
    public static double metersPerSecondToTicks(double input){
        return input * Constants.CONVERSION_METERS_SECOND_TO_TICKS;
    }
    public static double ticksToMetersPerSecond(double input){
        return input / Constants.CONVERSION_METERS_SECOND_TO_TICKS;
    }
}
