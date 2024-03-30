package frc.robot;

import org.ejml.equation.IntegerSequence.Range;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ELEVATOR;

public class RangeTable {
    public final static RangeEntry[] RANGE_TABLE = {
            // new RangeEntry(3000, 5), //Speaker
            // new RangeEntry(4500, 30.5) //Podium
            new RangeEntry(3500, 3500, 0.0), //0.0m
            new RangeEntry(3500, 3500, 0.0), //0.2m
            new RangeEntry(3500, 3500, 0.0), //0.4
            new RangeEntry(3500, 3500, 0.0), //0.6
            new RangeEntry(3500, 3500, 0.0), //0.8
            new RangeEntry(3500, 3500, 0.0), //1.0
            new RangeEntry(3500, 3500, 0.000), //1.2
            new RangeEntry(3500, 3500, 0.000), //1.4
            new RangeEntry(4000, 3500, 11.00), //1.6
            new RangeEntry(5000, 3500, 13.00), //1.8
            new RangeEntry(5500, 3500, 18.00), //2.0
            new RangeEntry(5500, 3500, 21.00), //2.2
            new RangeEntry(5500, 3500, 25.00), //2.4
            new RangeEntry(5500, 3500, 27.50), //2.6
            new RangeEntry(5500, 3500, 29.00), //2.8
            new RangeEntry(5500, 3500, 30.00), //3.0
            new RangeEntry(5500, 3500, 30.75), //3.2
            new RangeEntry(5500, 3500, 31.50), //3.4 xx
            new RangeEntry(5500, 3500, 32.25), //3.6 xx
            new RangeEntry(6000, 4000, 36.25), //3.8
            new RangeEntry(6000, 4000, 36.50), //4.0
            new RangeEntry(6000, 4000, 37.70), //4.2 *
            new RangeEntry(6000, 4000, 37.50), //4.4
            new RangeEntry(6000, 4000, 38.00), //4.6 
            new RangeEntry(6000, 4500, 39.00), //4.8
            new RangeEntry(6000, 4500, 40.00), //5.0 
            new RangeEntry(6000, 4500, 39.70), //5.2
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
            RangeEntry generated = new RangeEntry((int) (this.leftFlywheelSpeed + (leftSpeedUnit * value)),
                    (int) (this.rightFlywheelSpeed + (rightSpeedUnit * value)),
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
}
