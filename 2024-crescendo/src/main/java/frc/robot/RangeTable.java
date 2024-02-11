package frc.robot;


public class RangeTable {
    public final RangeEntry[] RANGE_TABLE = {
            new RangeEntry(0, 0) //TODO: Fill this out with real data
    };
    public boolean valid;

    public RangeTable() {
    }

    /**
     * Returns a RangeEntry representing the flywheel speed and pivot angle that should be used for the inputted distance
     * @param distance the distance to the target IN INCHES
     * @return a RangeEntry with flywheel speed in RPM and pivot angle in degrees. If the distance is too far, returns {@code null}; MAKE SURE TO CHECK FOR THIS
     */
    public RangeEntry get(double distance) {
        valid = true;
        int index = (int) (distance / 12);
        int inchesPastLastFoot = (int) (distance) % 12;
        if (RANGE_TABLE.length > 0) {
            if (index < RANGE_TABLE.length) { // Too far for the range table
                if ((int) (distance) % 12 == 0) {// If we're at a whole foot number
                    return RANGE_TABLE[index];
                } else if (index < RANGE_TABLE.length - 1) {
                    return RANGE_TABLE[index].interpolate(RANGE_TABLE[index + 1], inchesPastLastFoot / 12);
                } else {
                    valid = false;
                    return RANGE_TABLE[RANGE_TABLE.length - 1];
                }
            } else {
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


    public class RangeEntry {
        int flywheelSpeed;
        int pivotAngle;

        public RangeEntry(int speed, int angle) {
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
            int speedUnit = rEntry.flywheelSpeed - this.flywheelSpeed;
            int angleUnit = rEntry.pivotAngle - this.pivotAngle;
            return new RangeEntry((int) (this.flywheelSpeed + (speedUnit * value)),
                    (int) (this.pivotAngle + (angleUnit * value)));
        }

    }
}
