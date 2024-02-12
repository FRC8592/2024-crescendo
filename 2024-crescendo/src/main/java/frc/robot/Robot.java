package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.NewtonSwerve.Gyro.NewtonPigeon;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.autonomous.*;
import frc.robot.autonomous.autons.*;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;

import frc.robot.Constants.*;

public class Robot extends LoggedRobot {
    
    //Used in simulation
    public static Field2d FIELD = new Field2d();
    
    //Controllers
    private XboxController driverController;
    private XboxController operatorController;

    //Autonomous objects
    private BaseAuto currentAuto;
    private AutonomousSelector autoSelect;

    //Subsystem and hardware objects
    private NewtonPigeon pigeon;
    private Swerve swerve;
    private Shooter shooter;
    private Intake intake;
    private Elevator elevator;
    private LimelightTargeting noteLock;
    private PoseVision poseGetter;
    private LED leds;
    private Power power;

    //private double targetAngle = 0;


    @Override
    public void robotInit() {
        //AdvantageKit
        Logger.recordMetadata("Crescendo", "MyProject"); // Set a metadata value

        if (isReal()) { // If running on a real robot
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        }
        else { // If simulated
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
        Logger.start();

        
        driverController = new XboxController(CONTROLLERS.DRIVER_PORT);
        operatorController = new XboxController(CONTROLLERS.OPERATOR_PORT);
        autoSelect = new AutonomousSelector();
        //pigeon = new NewtonPigeon(new Pigeon2(PIGEON.CAN_ID));
        //swerve = new Swerve(pigeon);
        //power = new Power();
        //leds = new LED();
        //shooter = new Shooter();
        //poseGetter = new PoseVision();
        //intake = new Intake();
        //noteLock = new LimelightTargeting(NOTELOCK.LIMELIGHT_NAME, NOTELOCK.LOCK_ERROR, NOTELOCK.CAMERA_HEIGHT,
        //        NOTELOCK.kP, NOTELOCK.kI, NOTELOCK.kD);
        elevator = new Elevator();
        
    }

    @Override
    public void robotPeriodic() {

        //SmartDashboard.putNumber("target angle", targetAngle);
        
        SmartDashboard.putNumber("elevator position in ticks", elevator.getElevatorLength());
        SmartDashboard.putNumber("elevator position in rotations", elevator.getElevatorLength());

        SmartDashboard.putNumber("pivot position in angle", elevator.getPivotAngle());
        //SmartDashboard.putNumber("elevator position in ticks", elevator.getPivotAngle()*CONVERSIONS.ANGLE_DEGREES_TO_TICKS*CONVERSIONS.PIVOT_GEAR_RATIO);
        SmartDashboard.putNumber("elevator position in Rotations", (elevator.getPivotAngle()*CONVERSIONS.PIVOT_GEAR_RATIO)/360);

    }

    @Override
    public void autonomousInit() {
        // shooter.setAlliance(DriverStation.getAlliance().get());
        currentAuto = autoSelect.getSelectedAutonomous();
        currentAuto.addModules(swerve, elevator, intake, shooter, noteLock);
        currentAuto.initialize();
        swerve.resetEncoder();
        swerve.resetPose(currentAuto.getStartPose());
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setAutoCurrentLimit();
    }

    @Override
    public void autonomousPeriodic() {
        currentAuto.periodic();
    }

    @Override
    public void teleopInit() {
        // shooter.setAlliance(DriverStation.getAlliance().get());
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setTeleopCurrentLimit();
    }

    @Override
    public void teleopPeriodic() {
        //TODO: Move the auto-aim functions to driver controller, make sure no others are needed, and add in the logic for what they will do

        /* 1. Drivetrain movement
         *      a. Translate: DRIVER left stick X and Y
         *      b. Rotate: DRIVER right stick X
         *      c. Slow Mode: DRIVER right bumper (hold)
         *      d. Re-zero: DRIVER back
         * 
         * 2. Intaking:
         *      a. Auto-intake - <Button>:
         *          i. Turn to note
         *          ii. Drive to note
         *          iii. Spin the rollers to collect
         *          iv. Stop the rollers a set time after the beam-break sensors are triggered
         *          v. MAYBE: Add note collection detection based on current as fallback for the beam-breaks
         *      b. Manual intake - <button>
         *          i. Spin the rollers
         *          ii. Stop the rollers a set time after the beam-break sensors are triggered
         *          iii. MAYBE: Add note collection detection based on current as fallback for the beam-breaks
         * 
         * 3. Speaker scoring:
         *      a. Auto-shoot - <button>:
         *          i. Aim at the speaker based on april tags
         *          ii. Use a range table to set pivot angle
         *          iii. Use a range table to set shooter speed
         *          iv. Run the feeder wheels to shoot the note
         *          v. Lower the pivot so we can go under the stage
         *      b. Manual shoot - <button>:
         *          i. ASSUMPTION: Robot is up against the subwoofer
         *          ii. ASSUMPTION: Pivot is completely down
         *          iii. Run the shooter flywheels at a set RPM
         *          iv. Run the feeder wheels to shoot the note
         *      c. Pre-shoot - <button>:
         *          i. Aim at the speaker based on april tags
         *          ii. Use a range table to set pivot angle
         *          iii. Use a range table to set shooter speed
         *          iv. Lower the pivot when the button is let go
         * 
         * 4. Amp scoring
         *      a. Auto amp score - <button>:
         *          i. Turn the pivot to a set angle
         *          ii. Extend the elevator to the right length
         *          iii. Turn in the right direction
         *          iv. Strafe to the right position
         *          v. Run the shooter flywheels at a low speed
         *          vi. Drive to the amp
         *          vii. Run the feeder wheels to score the note into the amp
         *          viii. Retract the elevator to the home position
         *          ix. Rotate the pivot down into the home position
         *      b. Manual amp preparation - <button>:
         *          i. Turn the pivot to a set angle
         *          ii. Extend the elevator to the right length
         *      c. Manual amp score
         *          i. Run the shooter flywheels at a low speed
         *          ii. Run the feeder wheels to score the note into the amp
         *          iii. Retract the elevator to the home position
         *          iv. Rotate the pivot down into the home position
         * 
         * 5. Regurgitate
         *      a. Regurgitate forward - <button>
         *      b. Regurgitate back - <button>
         * 
         * 6. Stage
         *      a. Pre-stage - <button>:
         *          i. Extend the elevator to maximum height
         *          ii. Rotate the pivot to 90°
         *      b. Climb - <some variable control>:
         *          i. Move the elevator up and down based on the control
         * 
         * 7. Stow - <button>: Retract the elevator, then lower the pivot
         * 
         * 8. Protective features:
         *      a. Refuse to move the pivot under the stage
         *      b. Refuse to manual shoot when not facing towards the speaker and when not close enough to it (won't fly over)
         *      c. Disable pose-based features if the gyro yaw is too far from the MG Jetson system yaw
         *      d. Disable shooting when in the other wing
         *      e. <Add any other limiters for when something might cause a penalty>
         * 
         * 9. "Maybe" features
         *      a. Juke (rotate around a point outside the robot to avoid a defender)
         * 
         */
        //Basic driving controls
        double driveTranslateY = driverController.getLeftY();
        double driveTranslateX = driverController.getLeftX();
        double driveRotate = driverController.getRightX();
        boolean slowMode = driverController.getRightBumper();
        
        //Intakes TODO: Revise with drivers
        boolean autoIntake = driverController.getLeftTriggerAxis() > 0.1;
        boolean intaking = operatorController.getAButton();
        
        //Shooting TODO: Revise with drivers
        boolean prepareForShoot = operatorController.getLeftTriggerAxis() > 0.1;
        boolean manualShoot = operatorController.getBButton();
        boolean autoShoot = driverController.getRightTriggerAxis() > 0.1;
        
        //Amp TODO: Revise with drivers
        boolean ampPrep = operatorController.getXButton();
        boolean autoAmpScore = driverController.getRightTriggerAxis() > 0.1;
        boolean manualAmpScore = operatorController.getLeftBumper();
        
        //Stage TODO: Revise with drivers
        boolean preStage = operatorController.getYButton();
        double elevatorControl = operatorController.getPOV() == 0 ? 1 : (operatorController.getPOV() == 180 ? -1 : 0);
        
        //Other TODO: Revise with drivers
        boolean regurgitateBack = operatorController.getLeftBumper();
        boolean regurgitateFront = operatorController.getRightBumper();
        boolean stow = operatorController.getAButton();
        
        //Create a new ChassisSpeeds object with X, Y, and angular velocity from controller input
        ChassisSpeeds currentSpeeds;
        if (slowMode) { //Slow Mode slows down the robot for better precision & control
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                            driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                            driveRotate * SWERVE.ROTATE_POWER_SLOW * swerve.getMaxAngularVelo()),
                    swerve.getGyroscopeRotation());
        }
        else {
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                            driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                            driveRotate * SWERVE.ROTATE_POWER_FAST * swerve.getMaxAngularVelo()),
                    swerve.getGyroscopeRotation());
        }
        if (autoIntake) { //Drive to the nearest note and intake it
            double turnSpeed = noteLock.turnRobot(0, null /*pid  controller */, "tx", driveRotate,
                    elevatorControl);
            double forwardSpeed = noteLock.turnRobot(0, null /* pid controller */, "ty", driveRotate,
                    elevatorControl);
            intake.intakeNote(0, 0); //TODO  test intake with robotSpeed and real values in intakeNote
            if (intake.hasNote()) {
                intake.intakeNote(0, 0);
            }
        }
        else if (autoShoot) { //Aim at the speaker and shoot into it
            poseGetter.turnToAprilTag();
            //TODO range table
            elevator.setPivotAngleCustom(poseGetter.distanceToAprilTag(-1));
            shooter.setSpeedRangeTable(poseGetter.distanceToAprilTag(-1));
            if (shooter.isReady()) {//isReady returns whether the shooter angle and 
                //flywheel speeds are within a threshhold  of where we asked them to be
                shooter.shoot(); //runs the feeder wheels
                if (! shooter.hasNote()) {
                    shooter.stopFlywheels();
                    shooter.stopFeeders();
                    elevator.stow();
                }
            }
        }
        else if (autoAmpScore) {
            elevator.setElevatorLengthCustom(ELEVATOR.POSITION_AMP);
            double rotationSpeed = poseGetter.turnToAprilTag(); //amp aprilTag
            double xVelocity = poseGetter.strafeToAprilTag();
            shooter.shootVelocityMode(-1);
            double yVelocity = poseGetter.driveToAprilTag();
            currentSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationSpeed);
            if (poseGetter.distanceToAprilTag(-1) < -1) {
                shooter.shoot();
                currentSpeeds = new ChassisSpeeds();
                if (! shooter.hasNote()) {
                    elevator.stow();
                }
            }
        }
        else if (prepareForShoot) {
            poseGetter.turnToAprilTag();
            // TODO range table
            elevator.setPivotAngleCustom(poseGetter.distanceToAprilTag(-1));
            if (manualShoot) {
                shooter.shootVelocityMode(-1);
                if (shooter.isReady()) {// isReady returns whether the shooter angle and
                    // flywheel speeds are within a threshhold of where we asked them to be
                    shooter.shoot(); // runs the feeder wheels
                    if (!shooter.hasNote()) {
                        shooter.stopFlywheels();
                        shooter.stopFeeders();
                        elevator.stow();
                    }
                }
            }
        }
        else if (ampPrep) {
            elevator.ampPosition();
            if (manualAmpScore) {
                shooter.shootVelocityMode(-1); // flywheels at low speed
                shooter.shoot(); // feeder wheels
                elevator.stow();
            }
        }
        else if (preStage) {
            double currentElevatorPos = elevator.getElevatorLength();
            if (elevatorControl == 1.0) {
                double targetPosition = currentElevatorPos + 0.01; // TODO: set constant
                elevator.setElevatorLengthCustom(targetPosition);
            }
            else if (elevatorControl == -1.0) {
                double targetPosition = currentElevatorPos - 0.01; // TODO:
                elevator.setElevatorLengthCustom(targetPosition);
            }
            elevator.setPivotAngleCustom(90); // set to 90 degrees
        }
        else if (regurgitateBack) {
            shooter.setFeederSpeed(-1); // backwards
        }
        else if (regurgitateFront) {
            shooter.setFeederSpeed(-1); // forwards
        }
        swerve.drive(currentSpeeds);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
        elevator.resetEncoders();
    }

    @Override
    public void testPeriodic() {
        elevator.update();
        if (operatorController.getXButtonPressed()){
            elevator.stow();
        } else if (operatorController.getAButtonPressed()){
            elevator.ampPosition();
        }
        else if (operatorController.getYButtonPressed()){
            elevator.climbPosition();
        }

        if (operatorController.getBButtonPressed()){
            elevator.resetEncoders();
        }

        //Once Chain is Added
        /*elevator.update();
        if (operatorController.getXButton()){
            elevator.setPivotAngleCustom(35);
        }

        if (operatorController.getYButton()){
            elevator.setElevatorLengthCustom(4096);
        }

        if(operatorController.getAButtonPressed()){
            elevator.stow();
        }

        if(operatorController.getBButtonPressed()){
            elevator.zero();
        }*/

        
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}