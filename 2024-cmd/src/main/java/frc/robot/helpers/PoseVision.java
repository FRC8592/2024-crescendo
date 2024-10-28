package frc.robot.helpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseVision {
    private static PoseVision instance = null;
    public static PoseVision getInstance(){
        if(instance == null){
            throw new IllegalStateException("PoseVision must be instantiated before attempting to use it");
        }
        return instance;
    }
    public static PoseVision instantiate(double kP, double kI, double kD, double setpoint){
        if(instance != null){
            throw new IllegalStateException("PoseVision can't be instantiated twice");
        }
        instance = new PoseVision(kP, kI, kD, setpoint);
        return instance;
    }

    private double tag_x;
    private double tag_y;
    private double tag_z;
    private double tag_yaw; 
    private int curr_tag_id;

    private double tag2_x;
    private double tag2_y;
    private double tag2_z;
    private double tag2_yaw; 
    private int curr_tag2_id;

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

    private PoseVision(double kP, double kI, double kD, double setpoint) {
        // instantate PID controller given constants in constructor        
        visual_servo_pid = new PIDController(kP, kI, kD);
        visual_servo_pid.setIZone(APRILTAG_VISION.iZONE);
    }

    public boolean getVisionActive() {
        return SmartDashboard.getBoolean("jetson_active", false);
    }

    /* -- CURRENT TAG -- */
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

    public int getCurrTagID() {
        curr_tag_id = (int)SmartDashboard.getNumber("jetson_apriltag_id", 0.0);
        return curr_tag_id;
    }

    public double getCurrTagYaw() {
        // negative so that clockwise positive
        tag_yaw = -SmartDashboard.getNumber("jetson_apriltag_pitch", 0); // coordinate systems are weird
        return tag_yaw;
    }

    public boolean getTagInView() {
        boolean tagInView = SmartDashboard.getBoolean("jetson_tag_visible", false);
        return tagInView;
    }

    /* -- CURRENT TAG 2 (CHECK IF IN VIEW) -- */
    public double getCurrTag2X() {
        tag2_x = SmartDashboard.getNumber("jetson2_apriltag_x", 0.0);
        return tag2_x;
    }

    public double getCurrTag2Y() {
        tag2_y = SmartDashboard.getNumber("jetson2_apriltag_y", 0.0);
        return tag2_y;
    }

    public double getCurrTag2Z() {
        tag2_z = SmartDashboard.getNumber("jetson2_apriltag_z", 0.0);
        return tag2_z;
    }

    public int getCurrTag2ID() {
        curr_tag2_id = (int)SmartDashboard.getNumber("jetson2_apriltag_id", 0.0);
        return curr_tag2_id;
    }

    public double getCurrTag2Yaw() {
        tag2_yaw = SmartDashboard.getNumber("jetson2_apriltag_pitch", 0); // coordinate systems are weird
        return tag2_yaw;
    }

    public boolean getTag2InView() {
        boolean tagInView = SmartDashboard.getBoolean("jetson2_tag_visible", false);
        return tagInView;
    }

    /* -- ABSOLUTE POSITIONING */
    public double getX() {
        return SmartDashboard.getNumber("vision_x", 0.0);
    }

    public double getY() {
        return SmartDashboard.getNumber("vision_y", 0.0);
    }

    public double getZ() {
        return SmartDashboard.getNumber("vision_z", 0.0);
    }

    public double getYaw() {
        return SmartDashboard.getNumber("vision_yaw", 0.0);
    }

    public Pose2d getPose2d() {
        return new Pose2d(getX(), getY(), new Rotation2d(getYaw()));
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

        return Math.atan2(getCurrTagX(), getCurrTagZ()) - getCurrTagYaw();
    }

    /**
     * Assume we have a tag in view. 
     * 
     * @param servoTarget 0 for x, 1 for y, 2 for z, 3 for yaw, 4 for tx, see {@code VISUAL_SERVO_TARGETS}
     * @return Variable of interest (e.g. v_x, v_y, omega, ...)
     */
    public double visual_servo(int servoTarget, double limit, int tag_id, double defaultValue) {
        // check to see if we're looking at the tag and if it's the right one
        LinkedList<Integer> tag_ids = new LinkedList<Integer>();
        if (getTagInView()) {
            tag_ids.add(getCurrTagID());
        }
        if (getTag2InView()) {
            tag_ids.add(getCurrTag2ID());
        }

        if (!tag_ids.contains(tag_id)) {
            // not looking at tag
            return defaultValue;
        }

        boolean useTagOne = true;
        // find if tag or tag 2 is the one we're looking for
        if (getCurrTagID() == tag_id) {
            // tag 1 is the one we're looking for
            useTagOne = true;
        }
        else {
            // tag 2 is the one we're looking for
            useTagOne = false;
        }

        double curr_value;
        if (servoTarget == 0) {
            if (useTagOne) {
                curr_value = getCurrTagX();
            }
            else {
                curr_value = getCurrTag2X();
            }
        }
        else if (servoTarget == 1) {
            if (useTagOne) {
                curr_value = getCurrTagY();
            }
            else {
                curr_value = getCurrTag2Y();
            }
        }
        else if (servoTarget == 2) {
            if (useTagOne) {
                curr_value = getCurrTagZ();
            }
            else {
                curr_value = getCurrTag2Z();
            }
        }
        else if (servoTarget == 3) {
            if (useTagOne) {
                curr_value = getCurrTagYaw();
            }
            else {
                curr_value = getCurrTag2Yaw();
            }
        }
        else if (servoTarget == 4) {
            curr_value = getTagTx();
        }
        else {
            return 0.0;
        }

        double out = visual_servo_pid.calculate(curr_value, setpoint);
        out = Math.max(out, -limit);
        out = Math.min(out, limit);

        return out;
    }

    /**
     * Assume we have a tag in view. 
     * 
     * @param servoTarget 0 for x, 1 for y, 2 for z, 3 for yaw, 4 for tx, see {@code VISUAL_SERVO_TARGETS}
     * @param limit maximum output
     * @param valid_tag_ids tag IDs to consider (e.g 4,7 for stage)
     * @return Variable of interest (e.g. v_x, v_y, omega, ...)
     */
    public double visual_servo(int servoTarget, double limit, List<Integer> valid_tag_ids, double defaultValue) {
        // check to see if we're looking at the tag and if it's the right one
        LinkedList<Integer> visible_tag_ids = new LinkedList<Integer>();
        if (getTagInView()) {
            visible_tag_ids.add(getCurrTagID());
        }
        if (getTag2InView()) {
            visible_tag_ids.add(getCurrTag2ID());
        }

        int current_tag_id = -1;
        for (int target_tag_id : valid_tag_ids) { // for tag in tags we care about
            if (visible_tag_ids.contains(target_tag_id)) { // if we're not looking at the tag
                current_tag_id = target_tag_id;
            }
        }
        if (current_tag_id == -1) { // if we're not looking at any of the tags
            return defaultValue;
        }

        boolean useTagOne = true;
        // find if tag or tag 2 is the one we're looking for
        if (getCurrTagID() == current_tag_id) {
            // tag 1 is the one we're looking for
            useTagOne = true;
        }
        else {
            // tag 2 is the one we're looking for
            useTagOne = false;
        }

        double curr_value;
        if (servoTarget == 0) {
            if (useTagOne) {
                curr_value = getCurrTagX();
            }
            else {
                curr_value = getCurrTag2X();
            }
        }
        else if (servoTarget == 1) {
            if (useTagOne) {
                curr_value = getCurrTagY();
            }
            else {
                curr_value = getCurrTag2Y();
            }
        }
        else if (servoTarget == 2) {
            if (useTagOne) {
                curr_value = getCurrTagZ();
            }
            else {
                curr_value = getCurrTag2Z();
            }
        }
        else if (servoTarget == 3) {
            if (useTagOne) {
                curr_value = getCurrTagYaw();
            }
            else {
                curr_value = getCurrTag2Yaw();
            }
        }
        else if (servoTarget == 4) {
            curr_value = getTagTx();
        }
        else {
            return 0.0;
        }

        double out = visual_servo_pid.calculate(curr_value, setpoint);
        Logger.recordOutput(APRILTAG_VISION.LOG_PATH+"VisualServoPID-Output", out);
        out = Math.max(out, -limit);
        out = Math.min(out, limit);
        Logger.recordOutput(APRILTAG_VISION.LOG_PATH+"VS-PID-OutputClamped", out);

        return out;
    }
    /**
     * Returns the distance to the apriltag in ground plane
     * Returns -1 if tag ID not in view
     * @param id
     * @return distance to apriltag (meters)
     */
    public double distanceToAprilTag(int id) {
        // check if it's tag 1 or tag 2, first check if it's in view
        // return directly because we are more confidient in tag 1
        if (getTagInView() && getCurrTagID() == id) {
            // it's tag 1
            return getCurrTagZ();
        }
        else if (getTag2InView() && getCurrTag2ID() == id) {
            // it's tag 2
            return getCurrTag2Z();
        }
        else {
            return -1.0; // tag not in view
        }
    }

    /**
     * Returns the distance to the apriltag in ground plane
     * Returns -1 if tag ID not in view
     * @param ids list of tag IDs
     * @return distance to apriltag (meters)
     */
    public double distanceToAprilTag(List<Integer> ids) {
        // check if it's tag 1 or tag 2, first check if it's in view
        // return directly because we are more confidient in tag 1
        if (getTagInView() && ids.contains(getCurrTagID())) {
            // it's tag 1
            return getCurrTagZ();
        }    
        else if (getTag2InView() && ids.contains(getCurrTag2ID())) {
            // it's tag 2
            return getCurrTag2Z();
        }
        else {
            return -1.0; // tag not in view
        }
    }

    public double offsetFromAprilTag(List<Integer> ids) {
        // check if it's tag 1 or tag 2, first check if it's in view
        // return directly because we are more confidient in tag 1
        if (getTagInView() && ids.contains(getCurrTagID())) {
            // it's tag 1
            return getCurrTagX();
        }
        else if (getTag2InView() && ids.contains(getCurrTag2ID())) {
            // it's tag 2
            return getCurrTag2X();
        }
        else {
            return -1.0; // tag not in view
        }
    }

    public double rotationalOffsetFromAprilTag(List<Integer> ids) {
        // check if it's tag 1 or tag 2, first check if it's in view
        // return directly because we are more confidient in tag 1
        if (getTagInView()) {
            getCurrTagYaw();
            // it's tag 1
            return getCurrTagYaw();
        }
        else if (getTag2InView()) {
            // it's tag 2
            return getCurrTag2Yaw();
        }
        else {
            return 0; // tag not in view
        }
    }
}