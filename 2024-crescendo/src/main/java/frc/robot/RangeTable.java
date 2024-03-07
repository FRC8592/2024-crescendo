package frc.robot;

import org.littletonrobotics.junction.Logger;

public class RangeTable {
    public final static RangeEntry[] RANGE_TABLE = {
            new RangeEntry(3000, 5), //Speaker //TODO: Fill this out with real data
            new RangeEntry(4500, 30.5) //Podium
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
        //TODO: Uncomment when we get a real range table
        // valid = true;
        // int index = (int) (distance / 12);
        // int inchesPastLastFoot = (int) (distance) % 12;
        // if (RANGE_TABLE.length > 0) {
        //     if (index < RANGE_TABLE.length) {
        //         if ((int) (distance) % 12 == 0) {// If we're at a whole foot number
        //             return RANGE_TABLE[index];
        //         } else if (index < RANGE_TABLE.length - 1) {
        //             return RANGE_TABLE[index].interpolate(RANGE_TABLE[index + 1], inchesPastLastFoot / 12);
        //         } else {
        //             valid = false;
        //             return RANGE_TABLE[RANGE_TABLE.length - 1];
        //         }
        //     } else { // Too far for the range table
        //         valid = false;
        //         return RANGE_TABLE[RANGE_TABLE.length - 1];
        //     }
        // } else {
        //     valid = false;
        //     System.out.println("ERROR: Range table is empty");
        //     return new RangeEntry(0, 0);
        // }
        Logger.recordOutput("RangeTable/GetValue", distance);
        return RANGE_TABLE[(int) distance];
    }

    public boolean isValid() {
        return valid;
    }


    public static class RangeEntry {
        int flywheelSpeed;
        double pivotAngle;

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
            return new RangeEntry((int) (this.flywheelSpeed + (speedUnit * value)),
                    (int) (this.pivotAngle + (angleUnit * value)));
        }

    }
}
