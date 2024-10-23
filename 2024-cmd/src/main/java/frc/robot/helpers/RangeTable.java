package frc.robot.helpers;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class RangeTable {
    public final static RangeEntry[] RANGE_TABLE = {
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
        new RangeEntry(6000, 4000, 35.25),  // 4.0m
        new RangeEntry(6000, 4000, 36),     // 4.2
        new RangeEntry(6000, 4000, 37.50),  // 4.4
        new RangeEntry(6000, 4000, 38.00),  // 4.6
        new RangeEntry(6000, 4500, 39.00),  // 4.8
        new RangeEntry(6000, 4500, 40.00),  // 5.0
        new RangeEntry(6000, 4500, 39.70),  // 5.2
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
        new RangeEntry(6000, 4500, 39.70),  // Untuned
    };
    public static boolean valid;

    public RangeTable() {}

    /**
     * Get the best RangeEntry for a given distance to the speaker
     *
     * @param distance the distance in meters to the speaker april tag
     * @return the RangeEntry (interpolated between the two closest entries)
     */
    public static RangeEntry get(double distance) {
        distance*=5;
        Logger.recordOutput("CustomLogs/RangeTable/InputDistance", distance);
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
                    RangeEntry entry = RANGE_TABLE[wholeMeters]
                        .interpolate(RANGE_TABLE[wholeMeters + 1], decimal);

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
         * Linearly interpolates between this RangeEntry and another one. Assumes that this is
         * the smaller of the two.
         *
         * @param rEntry the range table entry to interpolate between
         * @param value where the result should be; 0 = this entry, 1 = the passed-in entry
         *
         * @return the interpolated entry
         */
        public RangeEntry interpolate(RangeEntry rEntry, double value) {
            double angleUnit = rEntry.pivotAngle - this.pivotAngle;

            RangeEntry generated = new RangeEntry(
                (int) (rEntry.leftFlywheelSpeed),
                (int) (rEntry.rightFlywheelSpeed),
                (this.pivotAngle + (angleUnit * value))
            );

            Logger.recordOutput(
                "CustomLogs/RangeTable/GeneratedPivotAngle",
                generated.pivotAngle
            );
            Logger.recordOutput(
                "CustomLogs/RangeTable/GeneratedFlywheelSpeed",
                generated.leftFlywheelSpeed
            );

            return generated;
        }
    }

    /**
     * @return the best entry for when we're pushed up against the subwoofer base
     */
    public static RangeEntry getSubwoofer() {
        return get(1.4);
    }

    /**
     * @return the best entry for when we're touching the podium and aiming at the speaker
     */
    public static RangeEntry getPodium() {
        // return get(2.83);
        return new RangeEntry(5250, 3250, 25.5, 0.0);
    }

    /**
     * @return the best entry to shoot into the trap
     */
    public static RangeEntry getTrap() {
        // return get(2.83);
        return new RangeEntry(3000, 3000, 5, 0.0);
    }
}
