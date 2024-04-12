package frc.robot;

import org.ejml.equation.IntegerSequence.Range;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ELEVATOR;

public class RangeTable {
    public final static RangeEntry[] RANGE_TABLE = {
            // new RangeEntry(3000, 5), //Speaker
            // new RangeEntry(4500, 30.5) //Podium
            new RangeEntry(3500, 3500, 0.0),    // 0.0m
            new RangeEntry(3500, 3500, 0.0),    // 0.2m
            new RangeEntry(3500, 3500, 0.0),    // 0.4m
            new RangeEntry(3500, 3500, 0.0),    // 0.6m
            new RangeEntry(3500, 3500, 0.0),    // 0.8m
            new RangeEntry(3500, 3500, 0.0),    // 1.0m
            new RangeEntry(3500, 3500, 0.0),    // 1.2m
            new RangeEntry(3500, 3500, 0.0),    // 1.4m
            new RangeEntry(3750, 3500, 5.5),    // 1.6m 
            new RangeEntry(4500, 3500, 12.5),   // 1.8m 
            new RangeEntry(5000, 3500, 18.5),   // 2.0m 
            new RangeEntry(5250, 3500, 21.5),   // 2.2m 
            new RangeEntry(5500, 3500, 25.0),   // 2.4m 
            new RangeEntry(5500, 3500, 28.0),   // 2.6m 
            new RangeEntry(5500, 3500, 29.5),   // 2.8m 
            new RangeEntry(5500, 3500, 30.0),   // 3.0m 
            new RangeEntry(5500, 3500, 30.5),   // 3.2m 
            new RangeEntry(5500, 3500, 31.25),  // 3.4m 
            new RangeEntry(5750, 3750, 32.0),   // 3.6m 
            new RangeEntry(6000, 4000, 34.625), // 3.8m 
            new RangeEntry(6000, 4000, 35.25),   // 4.0m
            new RangeEntry(6000, 4000, 36), //4.2 *
            new RangeEntry(6000, 4000, 37.50), //4.4
            new RangeEntry(6000, 4000, 38.00), //4.6 
            new RangeEntry(6000, 4500, 39.00), //4.8
            new RangeEntry(6000, 4500, 40.00), //5.0 
            new RangeEntry(6000, 4500, 39.70), //5.2
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
            new RangeEntry(6000, 4500, 39.70), //Untuned
    };
    public static boolean valid;

    public RangeTable() {
    }

    /**
     * @param distance TEMPORARY: index in the range table to return.
     * @return the RangeEntry at index {@code distance} in the range table
     */
    // /**
    // * Returns a RangeEntry representing the flywheel speed and pivot angle that
    // * should be used for the inputted distance
    // * @param distance the distance to the target IN INCHES
    // * @return a RangeEntry with flywheel speed in RPM and pivot angle in degrees.
    // * If the distance is too far, returns {@code null}; MAKE SURE TO CHECK FOR THIS
    // */
    public static RangeEntry get(double distance) {
        distance*=5;
        Logger.recordOutput("CustomLogs/RangeTable/InputDistance", distance);
        //TODO: Uncomment when we get a real range table
        valid = true;
        int wholeMeters = (int)(distance);
        if(distance <= 0){
            valid=false;
            return new RangeEntry(0,0,0);
        }
        double decimal = distance - Math.floor(distance);
        if (RANGE_TABLE.length > 0) {
            if (wholeMeters < RANGE_TABLE.length) {
                if (wholeMeters < RANGE_TABLE.length - 1) {
                    RangeEntry entry = RANGE_TABLE[wholeMeters].interpolate(RANGE_TABLE[wholeMeters + 1], decimal);
                    entry.pivotAngle += Constants.RANGE_TABLE.OFFSET_ANGLE;
                    entry.pivotAngle = Math.max(entry.pivotAngle, 0);
                    return entry;
                } else {
                    valid = false;
                    RangeEntry entry = RANGE_TABLE[RANGE_TABLE.length - 1];
                    entry.pivotAngle += Constants.RANGE_TABLE.OFFSET_ANGLE;
                    entry.pivotAngle = Math.max(entry.pivotAngle, 0);
                    return entry;
                }
            } else { // Too far for the range table
                valid = false;
                RangeEntry entry = RANGE_TABLE[RANGE_TABLE.length - 1];
                entry.pivotAngle += Constants.RANGE_TABLE.OFFSET_ANGLE;
                entry.pivotAngle = Math.max(entry.pivotAngle, 0);
                    return entry;
            }
        } else {
            valid = false;
            System.out.println("ERROR: Range table is empty");
            return new RangeEntry(0, 0, 0);
        }
    }

    public boolean isValid() {
        return valid;
    }


    public static class RangeEntry {
        public int leftFlywheelSpeed;
        public int rightFlywheelSpeed;
        public double pivotAngle;
        public double elevatorHeight;

        public RangeEntry(int left, int right, double angle) {
            leftFlywheelSpeed = left;
            rightFlywheelSpeed = right;
            pivotAngle = angle;
            elevatorHeight = 0;
        }

        public RangeEntry(int left, int right, double angle, double height) {
            leftFlywheelSpeed = left;
            rightFlywheelSpeed = right;
            pivotAngle = angle;
            elevatorHeight = height;
        }

        /**
         * Interpolates between this RangeEntry and another one. Assumes that this is the smaller of the two.
         * @param rEntry the range table entry to interpolate between
         * @param value
         * @return
         */
        public RangeEntry interpolate(RangeEntry rEntry, double value) {
            double leftSpeedUnit = rEntry.leftFlywheelSpeed - this.leftFlywheelSpeed;
            double rightSpeedUnit = rEntry.leftFlywheelSpeed - this.leftFlywheelSpeed;
            double angleUnit = rEntry.pivotAngle - this.pivotAngle;
            RangeEntry generated = new RangeEntry((int) (rEntry.leftFlywheelSpeed),
                    (int) (rEntry.rightFlywheelSpeed),
                    (this.pivotAngle + (angleUnit * value)));
            Logger.recordOutput("CustomLogs/RangeTable/GeneratedPivotAngle", generated.pivotAngle);
            Logger.recordOutput("CustomLogs/RangeTable/GeneratedFlywheelSpeed", generated.leftFlywheelSpeed);
            return generated;
        }
    }

    public static RangeEntry getKiddyPool() {
        return new RangeEntry(4500, 3500, 32, ELEVATOR.EXTENSION_METERS_MAX);
    }

    public static RangeEntry getSubwoofer() {
        return get(1.4);
    }

    public static RangeEntry getPodium() { // TODO:  Return this to a normal podium shot.  It has a temporary power boost.
        // return get(2.83);
        return new RangeEntry(5250, 3250, 27.5, 0.0);
    }

    public static RangeEntry getTrap() { // TODO:  Return this to a normal podium shot.  It has a temporary power boost.
        // return get(2.83);
        return new RangeEntry(3000, 3000, 5, 0.0);
    }
}
