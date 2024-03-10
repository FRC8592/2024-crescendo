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
    
    private PIDController targetingPID;

    public enum TargetVariable{
        LEFT_RIGHT_POSITION,
        UP_DOWN_POSITION,
        FORWARD_BACK_POSITION,
        LEFT_RIGHT_ROTATION
    }

    public PoseVision(double kP, double kI, double kD, double setpoint) { //TODO: Make separate PIDs for position targeting and rotation targeting
        // instantate PID controller given constants in constructor        
        targetingPID = new PIDController(kP, kI, kD);
    }

    //TODO: Units for all the getCurrTag_ functions
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
     * Assume we have a tag in view. 
     * 
     * @param servoTarget 1 for x, 2 for y, 3 for z, 4 for yaw, see {@code VISUAL_SERVO_TARGETS}
     * @return Variable of interest (e.g. v_x, v_y, omega, ...)
     */
    //TODO: This function won't work unless the Orange Pi magically knows to use the tag we want.
    //TODO: e.g. we ask for 4, and the Oak sees that tag, but getCurrTagID() returns 3 because
    //TODO: the Oak also sees 3 and 3 is the closer of the two. How do we specify which tag to look at?
    //TODO: Or do we have the processing power to get a list of tags and all their relative coordinates?
    public double target(TargetVariable servoTarget, double limit, int tag_id, double defaultValue) {
        // check to see if we're looking at the tag and if it's the right one
        if (getCurrTagID() != tag_id || !getTagInView()) {
            // not looking at tag
            return defaultValue;
        }

        double curr_value;
        switch(servoTarget){
            case LEFT_RIGHT_POSITION:
                curr_value = getCurrTagX();
                break;
            case UP_DOWN_POSITION:
                curr_value = getCurrTagY();
                break;
            case FORWARD_BACK_POSITION:
                curr_value = getCurrTagZ();
                break;
            case LEFT_RIGHT_ROTATION:
                curr_value = getCurrTagYaw();
            default:
                return 0.0; // TODO: better edge case behavior
        }

        double out = targetingPID.calculate(curr_value, setpoint);
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