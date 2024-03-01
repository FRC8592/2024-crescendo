package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;


import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PoseVision {
    private double tag_x;
    private double tag_y;
    private double tag_z;
    private double tag_yaw; 
    private double curr_tag_id;

    private double kP;
    private double kI;
    private double kD;
    private double setpoint;
    
    private PIDController visual_servo_pid;

    public String[] VISUAL_SERVO_TARGETS = {
        "RELATIVE_X", // left-right
        "RELATIVE_Y", // up-down (will probably never need this)
        "RELATIVE_Z", // forward distance to apriltag projected out from the tag or camera
        "RELATIVE_YAW" // e.g. even if mis-aligned, robot is pointing in same direction as apriltag
    };

    public PoseVision(double kP, double kI, double kD, double setpoint) {
        // instantate PID controller given constants in constructor        
        visual_servo_pid = new PIDController(kP, kI, kD);
    }

    public double getCurrTagX() {
        tag_x = SmartDashboard.getNumber("jetson_apriltag_x", 0.0);
        return tag_x;
    }

    public double getCurrTagY() {
        tag_y = SmartDashboard.getNumber("jetson_apriltag_y", 0.0);
        return tag_y;
    }

    public double getCurrTagZ() {
        tag_z = SmartDashboard.getNumber("jetson_apriltag_z", 0.0);
        return tag_z;
    }

    public double getCurrTagID() {
        curr_tag_id = SmartDashboard.getNumber("jetson_apriltag_id", 0.0);
        return curr_tag_id;
    }

    public double getCurrTagYaw() {
        tag_yaw = SmartDashboard.getNumber("jetson_apriltag_pitch", 0); // coordinate systems are weird
        return tag_yaw;
    }

    public boolean getTagInView() {
        boolean tagInView = SmartDashboard.getBoolean("jetson_tag_visible", false);
        return tagInView;
    }

    /**
     * Return "tx" equivalent from limelight
     * @return tx calculated from Oak-D data
     */
    public double getTagTx() {
        // atan2(z, x) - yaw  
        // verify math is correct

        // SOP log
        System.out.println("atan2(z,x) = " + Math.atan2(getCurrTagZ(), getCurrTagX()));
        System.out.println("yaw = " + getCurrTagYaw());

        return Math.atan2(getCurrTagZ(), getCurrTagX()) - getCurrTagYaw();
    }

    /**
     * Assume we have a tag in view. 
     * 
     * @param servoTarget 1 for x, 2 for y, 3 for z, 4 for yaw, 5 for tx, see {@code VISUAL_SERVO_TARGETS}
     * @return Variable of interest (e.g. v_x, v_y, omega, ...)
     */
    public double visual_servo(int servoTarget, double limit, int tag_id, double defaultValue) {
        // check to see if we're looking at the tag and if it's the right one
        // if (getCurrTagID() != tag_id || !getTagInView()) {
        //     // not looking at tag
        //     return defaultValue;
        // }

        double curr_value;
        if (servoTarget == 0) {
            curr_value = getCurrTagX();
        }
        else if (servoTarget == 1) {
            curr_value = getCurrTagY();
        }
        else if (servoTarget == 2) {
            curr_value = getCurrTagZ(); 
        }
        else if (servoTarget == 3) {
            curr_value = getCurrTagYaw();
        }
        else if (servoTarget == 4) {
            curr_value = getTagTx();
        }
        else {
            return 0.0; // TODO: better edge case behaviour
        }

        double out = visual_servo_pid.calculate(curr_value, setpoint);
        out = Math.max(out, -limit);
        out = Math.min(out, limit);

        return out;
    }

    /**
     * DEPRECATED: USE visual_servo() w/ input 3
     * @return
     */
    public double turnToAprilTag() {
        return 0.0;
    }

    /**
     * DEPRECATED: USE visual_servo() w/ input 0
     * @return
     */
    public double strafeToAprilTag() {
        return 0.0;
    }

    /**
     * DEPRECATED: USE visual_servo() w/ input 2
     * @return
     */
    public double driveToAprilTag() {
        return 0.0;
    }

    /**
     * Manav pls add info
     * @param id
     * @return
     */
    public double distanceToAprilTag(int id) {
        return 0.0;
    }
    
    /**
     * sets the alliance to blue or red!!
     * Manav plz fix this
     * @param alliance
     */
    public void setAlliance(Alliance alliance) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAlliance'");
    }
}