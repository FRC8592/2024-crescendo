package frc.robot;

import org.ejml.equation.IntegerSequence.Range;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.NewtonSwerve.Gyro.NewtonPigeon2;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
    private NewtonPigeon2 pigeon;
    private Swerve swerve;
    private Shooter shooter;
    private Intake intake;
    private Elevator elevator;
    private LimelightTargeting noteLock;
    private PoseVision apriltagLockYaw;
    private PoseVision apriltagLockY;
    private PoseVision poseGetter;
    private LED leds;
    private Power power;
    private PIDController turnPID;
    private PIDController drivePID;
    private SmoothingFilter smoothingFilter;
    private LimelightTargeting gameObjectVision;
    private boolean intaking;
    private boolean intakeToggleLastFrame = false;


    @Override
    public void robotInit() {

        //AdvantageKit
        Logger.recordMetadata("Crescendo", "MyProject"); // Set a metadata value

        if (isReal()) { // If running on a real robot
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
            Logger.start();
        }
        else { // If simulated
            // setUseTiming(false); // Run as fast as possible
            // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            SmartDashboard.putData(FIELD);
        }

        driverController = new XboxController(CONTROLLERS.DRIVER_PORT);
        operatorController = new XboxController(CONTROLLERS.OPERATOR_PORT);
        autoSelect = new AutonomousSelector();
        pigeon = new NewtonPigeon2(new Pigeon2(PIGEON.CAN_ID));
        swerve = new Swerve(pigeon);
        power = new Power();
        // leds = new LED();
        shooter = new Shooter();
        shooter.setMotorsIZone(SHOOTER.SHOOTER_MOTOR_IZONE);
        poseGetter = new PoseVision(APRILTAG_VISION.kP,APRILTAG_VISION.kI,APRILTAG_VISION.kD,0);
        intake = new Intake();
        noteLock = new LimelightTargeting(NOTELOCK.LIMELIGHT_NAME, NOTELOCK.LOCK_ERROR, NOTELOCK.CAMERA_HEIGHT,0,0,0);
        elevator = new Elevator();
        smoothingFilter = new SmoothingFilter(1, 1, 1);

        elevator.resetEncoders();

        drivePID = new PIDController(NOTELOCK.DRIVE_TO_DRIVE_kP, NOTELOCK.DRIVE_TO_DRIVE_kI, NOTELOCK.DRIVE_TO_DRIVE_kD);
        turnPID = new PIDController(NOTELOCK.DRIVE_TO_TURN_kP, NOTELOCK.DRIVE_TO_TURN_kI, NOTELOCK.DRIVE_TO_TURN_kD);

        SmartDashboard.putNumber("topShootSpeed", 4500);
        SmartDashboard.putNumber("bottomShootSpeed", 4500);
        SmartDashboard.putNumber("feederSpeedRPM", -500);
        SmartDashboard.putNumber("feederSpeedShootRPM", -1000);

        SmartDashboard.putNumber("PIVOT CUSTOM ANGLE", 45);

        SmartDashboard.putNumber("FeederKp", SHOOTER.FEEDER_MOTOR_kP);
        SmartDashboard.putNumber("FeederKi", SHOOTER.FEEDER_MOTOR_kI);
        SmartDashboard.putNumber("FeederKd", SHOOTER.FEEDER_MOTOR_kD);
        SmartDashboard.putNumber("FeederKff", SHOOTER.FEEDER_MOTOR_kF);

        SmartDashboard.putNumber("IntakeKp", INTAKE.TOP_MOTOR_kP);
        SmartDashboard.putNumber("IntakeKi", INTAKE.TOP_MOTOR_kI);
        SmartDashboard.putNumber("IntakeKd", INTAKE.TOP_MOTOR_kD);
        SmartDashboard.putNumber("IntakeKff",INTAKE.TOP_MOTOR_kFF);

        SmartDashboard.putBoolean("hasNote()", false);

        SmartDashboard.putBoolean("hasNote()", false);
    }

    @Override
    public void robotPeriodic() {

        Logger.recordOutput(SHOOTER.LOG_PATH+"TopMotorRPM", shooter.topShooterMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"BottomMotorRPM", shooter.bottomShooterMotor.getVelocity());

        Logger.recordOutput(ELEVATOR.LOG_PATH+"ExtensionMeters", elevator.getExtensionLength());
        Logger.recordOutput(ELEVATOR.LOG_PATH+"AngleDegrees", elevator.getPivotAngle());

        Logger.recordOutput(SHOOTER.LOG_PATH+"ShooterSpeedDifference (Bottom - Top)", shooter.bottomShooterMotor.getVelocity()-shooter.topShooterMotor.getVelocity());

        Logger.recordOutput(SHOOTER.LOG_PATH+"HasNote", shooter.noteBeamBreak.get());
        SmartDashboard.putBoolean("hasNote()", shooter.noteBeamBreak.get());

        //SmartDashboard.putNumber("target angle", targetAngle);
        elevator.update();

        
        // SmartDashboard.putNumber("elevator position in meters", elevator.getExtensionLength());
        // SmartDashboard.putNumber("elevator position in rotations", elevator.getExtensionLength()/ELEVATOR.ELEVATOR_GEAR_RATIO);

        // SmartDashboard.putNumber("pivot position in angle", elevator.getPivotAngle());
        //SmartDashboard.putNumber("elevator position in ticks", elevator.getPivotAngle()*CONVERSIONS.ANGLE_DEGREES_TO_TICKS*CONVERSIONS.PIVOT_GEAR_RATIO);
        // SmartDashboard.putNumber("elevator position in Rotations", (elevator.getPivotAngle()*ELEVATOR.PIVOT_GEAR_RATIO)/360);
        
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
        swerve.zeroGyroscope();
        swerve.drive(new ChassisSpeeds());
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

        intaking = false;

        // shooter.feederMotor.setPIDF(SmartDashboard.getNumber("FeederKp", 0),
        // SmartDashboard.getNumber("FeederKp", 0),
        // SmartDashboard.getNumber("FeederKp", 0),
        // SmartDashboard.getNumber("FeederKp", 0), 0);
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
        double driveTranslateY = -driverController.getLeftY();
        double driveTranslateX = -driverController.getLeftX();
        double driveRotate = -driverController.getRightX();
        boolean slowMode = driverController.getRightBumper();
        boolean resetGyro = driverController.getBackButtonPressed();
        boolean autoCollect = driverController.getLeftBumper();
        boolean robotOriented = driverController.getRightTriggerAxis() >0.1;

        //operator controls
        // shooter/feeder functions
        boolean shoot = operatorController.getRightTriggerAxis() > 0.1;
        boolean shootFromPodium = operatorController.getLeftBumper(); 

        boolean outake = operatorController.getRightBumper();
        boolean intakeToggle = operatorController.getLeftTriggerAxis() > 0.1 && !intakeToggleLastFrame;
        intakeToggleLastFrame = operatorController.getLeftTriggerAxis() > 0.1;

        boolean stowed = operatorController.getAButtonPressed();
        boolean ampPosition = operatorController.getXButton();
        boolean maxClimbPosition = operatorController.getYButtonPressed();
        boolean prime = operatorController.getBButtonPressed();
        boolean manualRaiseClimber = operatorController.getPOV() == 0;
        boolean manualLowerClimber = operatorController.getPOV() == 180;
        boolean manualPivotUp = operatorController.getPOV() == 90;
        boolean manualPivotDown = operatorController.getPOV() == 270;

        if (intakeToggle) {
            intaking = !intaking;
        }

        //Create a new ChassisSpeeds object with X, Y, and angular velocity from controller input
        ChassisSpeeds currentSpeeds;
        if(resetGyro){
            swerve.zeroGyroscope();
        }

        if (slowMode) { //Slow Mode slows down the robot for better precision & control
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                    driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                    driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                    driveRotate * SWERVE.ROTATE_POWER_SLOW * swerve.getMaxAngularVelo()));
        }
        else {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                    driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                    driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                    driveRotate * SWERVE.ROTATE_POWER_FAST * swerve.getMaxAngularVelo()));
        }
        currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(currentSpeeds, robotOriented?new Rotation2d():swerve.getGyroscopeRotation());
        noteLock.updateVision();

                               
        if(intaking){
            intake.spinPercentOutput(INTAKE.INTAKE_POWER);
            elevator.stow();
            if (shooter.hasNote()) {
                intaking = false;
            }
            else {
                shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED);
            }
        }

        else if(autoCollect){
            if (!shooter.hasNote()) {
                currentSpeeds = noteLock.driveToTarget(turnPID, drivePID, NOTELOCK.DRIVE_TO_TARGET_ANGLE);
                shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED);
                intake.spinPercentOutput(INTAKE.INTAKE_POWER);
                elevator.stow();
            }
        }

        else if(outake){
            intake.spinPercentOutput(INTAKE.OUTAKE_POWER);
            shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_VELOCITY);
            shooter.setShootVelocity(SHOOTER.OUTAKE_SHOOTER_VELOCITY, -SHOOTER.OUTAKE_SHOOTER_VELOCITY);
        }

        else if (shoot) { // TODO: Create shoot method to shoot from any distance 
            if (ampPosition) {
                shooter.setShootVelocity(Constants.SHOOTER.AMP_SHOOTER_SPEED, Constants.SHOOTER.AMP_SHOOTER_SPEED);
                // shooter.setShootVelocity(SHOOTER.AMP_SHOOTER_SPEED, SHOOTER.AMP_SHOOTER_SPEED);
                // shooter.setFeederVelocity(SHOOTER.AMP_FEEDER_SPEED);
                // shooter.hasNote = false;
            } else {
                RangeTable.RangeEntry entry = RangeTable.get(0);
                shooter.setShootVelocity(entry.flywheelSpeed,entry.flywheelSpeed);
                elevator.setPivotAngleCustom(entry.pivotAngle);
            }
            if (shooter.isReady()) {// isReady returns whether the shooter angle and flywheel speeds are within a threshhold of where we asked them to be.
                shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED); // runs the feeder wheels
            }
        }

        else if (shootFromPodium) { 
            RangeTable.RangeEntry entry = RangeTable.get(0);
            shooter.setShootVelocity(4500,4500); 
            elevator.setPivotAngleCustom(29.5);
            if (shooter.isReady() && elevator.isTargetAngle()) {// isReady returns whether the shooter angle and flywheel speeds are within a threshhold of where we asked them to be.
                shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED); // runs the feeder wheels
            }
        }

        else{
            if (stowed){
                elevator.stow();
            }
            else if (prime){
                elevator.setPivotAngleCustom(SmartDashboard.getNumber("PIVOT CUSTOM ANGLE", 5));
            }
            else if (ampPosition){
                elevator.ampPosition();
            }
            else if (maxClimbPosition) {
                elevator.climbPosition();
            }
            else if (manualRaiseClimber){
                elevator.extend();
            }
            else if (manualLowerClimber){
                elevator.retract();
            }
            else if (manualPivotUp){
                elevator.lift();
            }
            else if (manualPivotDown){
                elevator.lower();
            }
            else{
                intake.halt();
                shooter.stopFeeders();
                shooter.stop();
            }
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
        shooter.bottomShooterMotor.setPIDF(
            SmartDashboard.getNumber("ShooterKp",  0),
            SmartDashboard.getNumber("ShooterKi",  0),
            SmartDashboard.getNumber("ShooterKd",  0),
            SmartDashboard.getNumber("ShooterKff", 0), 0
        );

        shooter.topShooterMotor.setPIDF(
            SmartDashboard.getNumber("ShooterKp",  0),
            SmartDashboard.getNumber("ShooterKi",  0),
            SmartDashboard.getNumber("ShooterKd",  0),
            SmartDashboard.getNumber("ShooterKff", 0), 0
        );

        intake.topMotor.setPIDF(
            SmartDashboard.getNumber("IntakeKp",  0),
            SmartDashboard.getNumber("IntakeKi",  0),
            SmartDashboard.getNumber("IntakeKd",  0),
            SmartDashboard.getNumber("IntakeKff", 0), 0
        );

        // shooter.setAlliance(DriverStation.getAlliance().get());
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setTeleopCurrentLimit();

        SmartDashboard.putNumber("Measured Intake Top RPM", 0);
        SmartDashboard.putNumber("Measured Intake Bottom RPM", 0);
    }

    @Override
    public void testPeriodic() {
        driverController.setRumble(RumbleType.kBothRumble, driverController.getLeftTriggerAxis() * 255);
        operatorController.setRumble(RumbleType.kBothRumble, operatorController.getLeftTriggerAxis() * 255);
    } 

    public void testDrivetrain(){
        
        //driver controls
        double driveTranslateY = -driverController.getLeftY();
        double driveTranslateX = -driverController.getLeftX();
        double driveRotate = driverController.getRightX();
        boolean slowMode = driverController.getRightBumper();
        boolean resetGyro = driverController.getBackButtonPressed();
        boolean robotOriented = driverController.getRightTriggerAxis() >0.1;
        // boolean driveToNote = driverController.getLeftTriggerAxis() >0.1;

        // Resets the gyroscope to 0 once b button is pressed
        if(resetGyro){
            swerve.zeroGyroscope();
        }

        //Create a new ChassisSpeeds object with X, Y, and angular velocity from controller input
        ChassisSpeeds currentSpeeds;
        if (slowMode) { //Slow Mode slows down the robot for better precision & control
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                            driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                            driveRotate * SWERVE.ROTATE_POWER_SLOW * swerve.getMaxAngularVelo()),
                    !robotOriented?swerve.getGyroscopeRotation():new Rotation2d());
        }
        else {
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                            driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                            driveRotate * SWERVE.ROTATE_POWER_FAST * swerve.getMaxAngularVelo()),
                    !robotOriented?swerve.getGyroscopeRotation():new Rotation2d());
        }
        currentSpeeds = smoothingFilter.smooth(currentSpeeds);
        swerve.drive(currentSpeeds);

    }

    public void testIntake(){
        
        //Intaking and Outaking controls
        boolean outake = operatorController.getRightBumper();
        boolean intaking = operatorController.getLeftTriggerAxis() > 0.1; // TODO: use dedicated deadband function
        boolean fullPowerIntake = operatorController.getLeftBumper();

         // intaking
        if (intaking) {
            // intake.spinPercentOutput(0.5);
            intake.setIntakeVelocity(SmartDashboard.getNumber("Intake Top RPM", 0));
        }
        else if(outake){
            intake.setIntakeVelocity(-2000);
            shooter.setFeederVelocity(2000);
            shooter.setShootVelocity(-2000, -2000);

        }
        // else if (fullPowerIntake) {
        //     intake.spinPercentOutput(1.0);
        // }
        else {
            intake.halt();
        }

    }

    public void testShoot(){
        boolean shoot = operatorController.getRightTriggerAxis() > 0.1;
        boolean runFeeder = operatorController.getLeftBumper();

         if (shoot) {
            shooter.setShootVelocity((int) SmartDashboard.getNumber("topShootSpeed", 0), 
                    (int) SmartDashboard.getNumber("bottomShootSpeed", 0));
            if (shooter.isReady()) {// isReady returns whether the shooter angle and
                // flywheel speeds are within a threshhold of where we asked them to be
                shooter.setFeederVelocity(SmartDashboard.getNumber("feederSpeedShootRPM", 0)); // runs the feeder wheels
                // if (!shooter.hasNote()) {
                //     shooter.stop();
                //     shooter.stopFeeders();
                //     elevator.stow()
                // }
            }
        } else if (runFeeder){
            shooter.setFeederVelocity(SmartDashboard.getNumber("feederSpeedRPM", 0));
        }
        else {
            shooter.setShootVelocity(0, 0);
            shooter.setFeederSpeed(0);
        }
    }

    // public void testElevator(){
        
    //     //elevator controls
    //     boolean stowed = operatorController.getAButtonPressed();
    //     boolean ampPosition = operatorController.getXButtonPressed();
    //     boolean maxClimbPosition = operatorController.getYButtonPressed();
    //     boolean prime = operatorController.getBButtonPressed();
    //     boolean manualRaiseClimber = operatorController.getPOV() == 0;
    //     boolean manualLowerClimber = operatorController.getPOV() == 180;
    //     boolean manualPivotUp = operatorController.getPOV() == 90;
    //     boolean manualPivotDown = operatorController.getPOV() == 270;
       

    //     if (stowed){
    //         elevator.stow();
    //     }
    //     else if (prime){
    //         elevator.setPivotAngleCustom(SmartDashboard.getNumber("PIVOT CUSTOM ANGLE", 0));
    //     }
    //     else if (ampPosition){
    //         elevator.ampPosition();
    //     }
    //     else if (maxClimbPosition) {
    //         elevator.climbPosition();
    //     }
    //     else if (manualRaiseClimber){
    //         manualExtensionLength += 0.0001; //meters
    //         elevator.setExtensionLengthCustom(manualExtensionLength);
    //     }
    //     else if (manualLowerClimber){
    //         manualExtensionLength -= 0.0001; //meters
    //         elevator.setExtensionLengthCustom(manualExtensionLength);
    //     }
    //     else if (manualPivotUp){
    //         manualPivotAngle += 0.1; //degrees
    //         elevator.setPivotAngleCustom(manualPivotAngle);
    //     }
    //     else if (manualPivotDown){
    //         manualPivotAngle -= 0.1; //degrees
    //         elevator.setPivotAngleCustom(manualPivotAngle);
    //     }
        
        // }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}