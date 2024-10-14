package frc.robot.helpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CONVERSIONS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

import java.util.LinkedList;

import org.littletonrobotics.junction.Logger;

public class LimelightTargeting {
    // constants passed in during initilization
    private double lockError;
    private double closeError;
    private double cameraHeight;
    private double cameraAngle;
    private double targetHeight;
    private double rotationKP;
    private double rotationKI;
    private double rotationKD;
    private double closeRotationKP;
    private double closeRotationKI;
    private double closeRotationKD;
    // Network Table entries
    public NetworkTableEntry tx; // Angle error (x) from LimeLight camera
    public NetworkTableEntry ty; // Angle error (y) from LimeLight camera
    public NetworkTableEntry ta; // Target area measurement from LimeLight camera
    public NetworkTableEntry tv; // Target valid indicator from Limelight camera
    // Shared variables
    public boolean targetValid; // Indicate when the Limelight camera has found a target
    public boolean targetLocked; // Indicate when the turret is centered on the target
    public boolean targetClose; // Indicate when the robot is close to centered on the target
    public double targetRange; // Range from robot to target (inches)
    public Timer timer;
    public double processedDx = 0;
    public double processedDy = 0;
    // Private autoaim variables
    private double turnSpeed;
    private double lastTime = 0;
    private double xtime = 0;
    private double lastAngle = 0;
    private double changeInAngleError = 0;

    // constants for averaging limelight averages
    private static int MIN_LOCKS = 1;
    private static int STAT_SIZE = 10;

    // Avoid driving forward if the angle error exceeds this value
    private static double MAX_ANGLE_ERROR_TO_DRIVE = 2.0;

    private LinkedList<LimelightData> previousCoordinates;

    private String limelightName;

    private double optDistance;
    private double distanceFeet;


    /**
     * This constructor will intialize internal variables for the robot turret
     */
    public LimelightTargeting(String limelightName, double lockError, double closeError,
            double cameraHeight, double cameraAngle, double targetHeight) {

        // Set up networktables for limelight
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");

        previousCoordinates = new LinkedList<LimelightData>();

        // Establish initial values for variables we share
        targetValid = false;
        targetLocked = false;
        targetClose = false;
        targetRange = 0.0;
        timer = new Timer();
        timer.start();

        this.limelightName = limelightName;
        this.lockError = lockError;
        this.closeError = closeError;
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
        this.targetHeight = targetHeight;

        // Creat the PID controller for turning

    }

    //
    // Reset internal variables to a benign state
    //
    public void reset() {
        targetValid = false;
        targetLocked = false;
        targetClose = false;
    }

    public void updateVision() { // method should be called continuously during autonomous and teleop
        double xError;
        double yError;
        double area;
        double totalDx = 0;
        double totalDy = 0;
        int totalValid = 0;

        // Read the Limelight data from the Network Tables
        xError = tx.getDouble(0.0);
        yError = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        targetValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean

        // generates average of limelight parameters
        previousCoordinates.add(new LimelightData(xError, yError, targetValid));
        if (previousCoordinates.size() > STAT_SIZE) {
            previousCoordinates.removeFirst();
        }

        for (LimelightData data : previousCoordinates) {
            if (data.ballValid == true) {
                totalDx = data.dx + totalDx;
                totalDy = data.dy + totalDy;
                totalValid = totalValid + 1;
            }
        }

        if (totalValid != 0) {
            processedDx = (totalDx / totalValid);
            processedDy = totalDy / totalValid;
        } else {
            processedDx = -1;
            processedDy = -1;
        }

        targetValid = (totalValid >= MIN_LOCKS);

        targetRange = distanceToTarget();

        if (Math.abs(processedDx) < lockError) { // Turret is pointing at target (or no target)
            targetLocked = targetValid; // We are only locked when targetValid
        } else {
            targetLocked = false;
        }

        if (Math.abs(processedDx) < closeError) { // Turret is close to locking
            targetClose = targetValid; // We are only close when targetValid
        } else {
            targetClose = false;
        }

        // post driver data to smart dashboard periodically
        // SmartDashboard.putNumber(limelightName + "/xerror in radians",
        // Math.toRadians(xError));
        // SmartDashboard.putNumber(limelightName + "/LimelightX", xError);
        // SmartDashboard.putNumber(limelightName + "/LimelightY", yError);
        // SmartDashboard.putNumber(limelightName + "/LimelightArea", area);
        // SmartDashboard.putBoolean(limelightName + "/Target Valid", targetValid);
        // SmartDashboard.putNumber(limelightName + "/Change in Angle Error",
        // changeInAngleError);
        // SmartDashboard.putNumber(limelightName + "/Average Y", processedDy);
        // SmartDashboard.putNumber(limelightName + "/Average X", processedDx);
        // SmartDashboard.putNumber(limelightName + "/Total Valid", totalValid);
        // SmartDashboard.putNumber(limelightName + "/Target Range", targetRange);
        // SmartDashboard.putBoolean(limelightName + "/inRange", targetRange >120 &&
        // targetRange < 265);
        // SmartDashboard.putBoolean(limelightName + "/Target Locked", targetLocked);
        // SmartDashboard.putBoolean(limelightName + "/Target Close", targetClose);
        // SmartDashboard.putNumber(limelightName + "/lockError", lockError);
    }

    public double delta() { // gets the change in angle over time(seconds)
        xtime = timer.get();
        changeInAngleError = (processedDx - lastAngle) / (xtime - lastTime);
        lastAngle = processedDx; // reset initial angle
        lastTime = xtime; // reset initial time
        return changeInAngleError;
    }

    /**
     * Gives distance from the robot to the target in inches
     * Com
     * ute range to target.
     * Formula taken from
     * https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
     *
     * @return distance in inches
     */
    public double distanceToTarget() {
        if (targetValid) {
            double distanceInches = (targetHeight - cameraHeight) /
                    Math.tan((cameraAngle + processedDy) *
                    CONVERSIONS.DEG_TO_RAD); // Equation is from limelight documentation finding distance

            return distanceInches;
        }
        return -1;
    }

    /**
     *
     * @return angle offset in radians
     */
    public double offsetAngle() {
        return Math.toRadians(processedDx);
    }

    public double getOffsetAngleDegrees() {
        return processedDx;
    }

    /**
     * Turn the robot based on limelight data
     * 1) If no targetValid turn in a circle until we get a targetValid indicator
     * 2) if targetValid, turns towards the target using the PID controller output
     * for turn speed
     * 3) if targetValid and processedDx is within our "locked" criteria, stop
     * turning
     *
     * @return The turn speed
     */
    public double turnRobot(double visionSearchSpeed, PIDController turnPID, String variable, double limit, double offset) {
        if (targetValid) {
            if (variable == "tx")
                turnSpeed = turnPID.calculate(processedDx, offset); // Setpoint is always 0 degrees (dead center)
            else if (variable == "ty")
                turnSpeed = turnPID.calculate(processedDy, offset); // Setpoint is always 0 degrees (dead center)
            turnSpeed = Math.max(turnSpeed, -limit);
            turnSpeed = Math.min(turnSpeed, limit);
        }

        // If no targetValid, spin in a circle to search
        else {
            turnSpeed = visionSearchSpeed; // Spin in a circle until a target is located
        }

        return turnSpeed;
    }

    
    public ChassisSpeeds driveToTarget(PIDController turnPID, PIDController drivePID, double targetAngle) {
        double rotateSpeed = this.turnRobot(0, turnPID, "tx", SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND, 0);
        double driveToSpeed = -this.turnRobot(0, drivePID, "ty", 4.5, targetAngle);

        //
        // Don't start driving forward until the target is near to the center of the image.
        // This will prevent the turn PID from having to make big last-minute adjustments
        // when we are close to the target, but have a large angle error.
        // 
        if (Math.abs(processedDx) > MAX_ANGLE_ERROR_TO_DRIVE)
            driveToSpeed = 0.0;

        SmartDashboard.putNumber("Drive-to velocity", driveToSpeed);
        Logger.recordOutput(NOTELOCK.LOG_PATH+"Drive-to Velocity", driveToSpeed);
        Logger.recordOutput(NOTELOCK.LOG_PATH+"RotateSpeed", rotateSpeed);
        return new ChassisSpeeds(driveToSpeed, 0, rotateSpeed);
    }

    /**
     *
     * @return true if vision target is locked within the allowed angular error
     */
    public boolean isTargetLocked() {
        return targetLocked;
    }

    public boolean isTargetValid() {
        return targetValid;
    }

    /**
     * Local class used to store a history of limelight data to allow averaging
     */
    private class LimelightData {
        double dx;
        double dy;
        boolean ballValid;

        public LimelightData(double xError, double yError, boolean ballValid) {
            dx = xError;
            dy = yError;
            this.ballValid = ballValid;
        }

    }

    public String getVisionName() {
        return this.limelightName;
    }
}