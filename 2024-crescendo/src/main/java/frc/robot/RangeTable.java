package frc.robot;

import org.littletonrobotics.junction.Logger;

public class RangeTable {
    public final static RangeEntry[] RANGE_TABLE = {
            // new RangeEntry(3000, 5), //Speaker
            // new RangeEntry(4500, 30.5) //Podium
            new RangeEntry(0, 0), //0m
            new RangeEntry(0, 0), //0.2m
            new RangeEntry(0, 0), //0.4
            new RangeEntry(0, 0), //0.6
            new RangeEntry(0, 0), //0.8
            new RangeEntry(0, 0), //1.0
            new RangeEntry(0, 0), //1.2
            new RangeEntry(0, 0), //1.4
            new RangeEntry(0, 0), //1.6
            new RangeEntry(4000, 19), //1.8
            new RangeEntry(4000, 21), //2.0
            new RangeEntry(4000, 25), //2.2
            new RangeEntry(4000, 27.5), //2.4
            new RangeEntry(4000, 30), //2.6
            new RangeEntry(4500, 32), //2.8
            new RangeEntry(4500, 34), //3.0
            new RangeEntry(4750, 34.75), //3.2
            new RangeEntry(4750, 36), //3.4
            new RangeEntry(4750, 37), //3.6
            new RangeEntry(4750, 37), //3.8
            new RangeEntry(0, 0), //4.0
            new RangeEntry(0, 0), //4.2
            new RangeEntry(0, 0),
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
            return new RangeEntry(0,0);
        }
        double decimal = distance - Math.floor(distance);
        if (RANGE_TABLE.length > 0) {
            if (wholeMeters < RANGE_TABLE.length) {
                if (wholeMeters < RANGE_TABLE.length - 1) {
                    return RANGE_TABLE[wholeMeters].interpolate(RANGE_TABLE[wholeMeters + 1], decimal);
                } else {
                    valid = false;
                    return RANGE_TABLE[RANGE_TABLE.length - 1];
                }
            } else { // Too far for the range table
                valid = false;
                return RANGE_TABLE[RANGE_TABLE.length - 1];
            }
        } else {
            valid = false;
            System.out.println("ERROR: Range table is empty");
            return new RangeEntry(0, 0);
        }
    }

    public boolean isValid() {
        return valid;
    }


    public static class RangeEntry {
        public int flywheelSpeed;
        public double pivotAngle;

        public RangeEntry(int speed, double angle) {
            flywheelSpeed = speed;
            pivotAngle = angle;
        }

        /**
         * Interpolates between this RangeEntry and another one. Assumes that this is the smaller of the two.
         * @param rEntry the range table entry to interpolate between
         * @param value
         * @return
         */
        public RangeEntry interpolate(RangeEntry rEntry, double value) {
            double speedUnit = rEntry.flywheelSpeed - this.flywheelSpeed;
            double angleUnit = rEntry.pivotAngle - this.pivotAngle;
            RangeEntry generated = new RangeEntry((int) (this.flywheelSpeed + (speedUnit * value)),
                    (this.pivotAngle + (angleUnit * value)));
            Logger.recordOutput("CustomLogs/RangeTable/GeneratedPivotAngle", generated.pivotAngle);
            Logger.recordOutput("CustomLogs/RangeTable/GeneratedFlywheelSpeed", generated.flywheelSpeed);
            return generated;
        }
    }
}
