package frc.robot;

import org.ejml.equation.IntegerSequence.Range;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.opencv.features2d.FlannBasedMatcher;

import com.NewtonSwerve.DriveController;
import com.NewtonSwerve.Gyro.NewtonPigeon2;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.autonomous.*;
import frc.robot.autonomous.autons.*;
import frc.robot.RangeTable.RangeEntry;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;

import frc.robot.Constants.*;
import frc.robot.MainSubsystemsManager.MechanismState;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

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
    // private LimelightTargeting apriltagVision;
    // private PoseVision apriltagLockYaw;
    // private PoseVision apriltagLockY;
    // private PoseVision poseGetter;
    private NeoPixelLED leds;
    private Power power;
    private PoseVision poseVision;
    private PIDController turnPID;
    private PIDController drivePID;
    private SmoothingFilter smoothingFilter;
    private MainSubsystemsManager subsystemsManager;

    public static RangeEntry currentRange;

    private Controls controls;
    
    // Keep track of the last commanded yaw and lock it in
    private Boolean yawLock = false;
    private double yawLockValue = 0;

    private double distance;
    public boolean locked;
    
    private Alliance currentAlliance = Alliance.Red;

    @Override
    public void robotInit() {

        //AdvantageKit
        Logger.recordMetadata("Crescendo", "MyProject"); // Set a metadata value
        
        if (isReal()) { // If running on a real robot
            String eventName = DriverStation.getEventName() != ""?DriverStation.getEventName()+"_":"";
            String matchType;
            switch (DriverStation.getMatchType()) {
                default: // Default falls through to None
                case None:
                    matchType = "";
                    break;
                case Practice:
                    matchType = "PRACTICE_";
                    break;
                case Qualification:
                    matchType = "QUALS_";
                    break;
                case Elimination:
                    matchType = "ELIM_";
                    break;
            }
            String matchNumber = DriverStation.getMatchNumber()>0?String.valueOf(DriverStation.getMatchNumber())+"_":"";
            String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());
            String path = "/U/"+eventName+matchType+matchNumber+time+".wpilog";
            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick
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
        // SmartDashboard.putNumber("Elevator Custom Angle", 0);
        // SmartDashboard.putNumber("Shooter Left Speed", 0);
        // SmartDashboard.putNumber("Shooter Right Speed", 0);

        driverController = new XboxController(CONTROLLERS.DRIVER_PORT);
        operatorController = new XboxController(CONTROLLERS.OPERATOR_PORT);
        autoSelect = new AutonomousSelector();

        pigeon = new NewtonPigeon2(new Pigeon2(CAN.PIGEON_CAN_ID));
        swerve = new Swerve(pigeon);

        power = new Power();
        leds = new NeoPixelLED();
        shooter = new Shooter();
        // poseGetter = new PoseVision(APRILTAG_VISION.kP,APRILTAG_VISION.kI,APRILTAG_VISION.kD,0);
        intake = new Intake();
        noteLock = new LimelightTargeting(NOTELOCK.LIMELIGHT_NAME, NOTELOCK.LOCK_ERROR,0,0,0,0);
        elevator = new Elevator();
        smoothingFilter = new SmoothingFilter(SWERVE.TRANSLATION_SMOOTHING_AMOUNT, SWERVE.TRANSLATION_SMOOTHING_AMOUNT, SWERVE.ROTATION_SMOOTHING_AMOUNT);

        elevator.resetEncoders();

        drivePID = new PIDController(APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kP, APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kI, APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kD);
        turnPID = new PIDController(NOTELOCK.DRIVE_TO_TURN_kP, NOTELOCK.DRIVE_TO_TURN_kI, NOTELOCK.DRIVE_TO_TURN_kD);

        poseVision = new PoseVision(APRILTAG_VISION.kP, APRILTAG_VISION.kI, APRILTAG_VISION.kD, 0);

        currentRange = new RangeEntry(0, 0, 0);
        subsystemsManager = new MainSubsystemsManager(intake, shooter, elevator, leds);

        this.controls = new Controls();

        // Rumble.init();
    }

    @Override
    public void robotPeriodic() {
        Logger.recordOutput(ELEVATOR.LOG_PATH+"ExtensionMeters", elevator.getExtensionLength());
        Logger.recordOutput(ELEVATOR.LOG_PATH+"AngleDegrees", elevator.getPivotAngle());

        Logger.recordOutput(ELEVATOR.LOG_PATH+"IsTargetAngle", elevator.isTargetAngle());
        Logger.recordOutput(ELEVATOR.LOG_PATH+"isTargetLength", elevator.isTargetLength());

        SmartDashboard.putBoolean("Top Beam Break", shooter.topBeamBreak.get());
        SmartDashboard.putBoolean("Bottom Beam Break", shooter.bottomBeamBreak.get());
        SmartDashboard.putBoolean("Middle Beam Break", shooter.middleBeamBreak.get());

        Logger.recordOutput(INTAKE.LOG_PATH+"IntakeVelocityRPM", intake.getTopMotorVelocityRPM());

        Logger.recordOutput(ELEVATOR.LOG_PATH+"PivotITerm",elevator.pivotMotor.motorControl.getIAccum());
        Logger.recordOutput(SWERVE.LOG_PATH+"Robot position", swerve.getCurrentPos());
        
        SmartDashboard.putNumber("Robot Yaw", swerve.getGyroscopeRotation().getDegrees());

        //SmartDashboard.putNumber("target angle", targetAngle);
        elevator.update();
        shooter.log();

        // update odometry with pose
        // swerve.addVisionMeasurement(poseVision);

        // NOTE: FOR TESTING PURPOSES. 
        /*
        if(poseVision.getTagInView() && poseVision.getCurrTagID() == 4) {
                SmartDashboard.putNumber("Tag 4 Z", poseVision.getCurrTagZ());
        }
        else if (poseVision.getTag2InView() && poseVision.getCurrTag2ID() == 4) {
            SmartDashboard.putNumber("Tag 4 Z", poseVision.getCurrTag2Z());
        }
        else {
            SmartDashboard.putNumber("Tag 4 Z", -1.0);
        }
        */
        Logger.recordOutput("Robot Pose from MGVision", poseVision.getPose2d());
        Logger.recordOutput(APRILTAG_VISION.LOG_PATH+"X offset (m)", poseVision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS));

        // Rumble.update(driverController, operatorController);

        // April Tag 
        distance = poseVision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS);
        locked = poseVision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS) < APRILTAG_VISION.X_ROT_LOCK_ERROR;

    }

    @Override
    public void autonomousInit() {
        // shooter.setAlliance(DriverStation.getAlliance().get());
        swerve.zeroGyroscope();
        currentAuto = autoSelect.getSelectedAutonomous();
        subsystemsManager.resetToLoaded();
        currentAuto.addModules(swerve, elevator, intake, shooter, noteLock, poseVision, subsystemsManager);
        currentAuto.initialize();
        currentAuto.addDelay(autoSelect.getDelay());
        swerve.resetEncoder();
        swerve.resetPose(currentAuto.getStartPose());
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setThrottleCurrentLimit(POWER.SWERVE_AUTO_THROTTLE_CURRENT_LIMIT);
        swerve.drive(new ChassisSpeeds());
    }

    @Override
    public void autonomousPeriodic() {
        currentRange = new RangeEntry(0, 0, 0);
        leds.solidColor(LEDS.OFF);
        currentAuto.periodic();
        leds.update(0, false);
        // subsystemsManager.updateMechanismStateMachine(controls, distance, locked); //`controls` is only updated in teleop, so MSM basically only responds to the state-setter used in the commands
    }

    @Override
    public void teleopInit() {
        // shooter.setAlliance(DriverStation.getAlliance().get());
        swerve.setSteerAnglesToAbsEncoder();
        //swerve.zeroGyoscope();
        swerve.setThrottleCurrentLimit(POWER.SWERVE_TELEOP_THROTTLE_CURRENT_LIMIT);
        leds.solidColor(LEDS.OFF);
        currentAlliance = DriverStation.getAlliance().get();
        if(currentAuto != null){
            swerve.setGyroscopeRotation(currentAuto.getStartPose().getRotation().getDegrees()
                    - swerve.getGyroscopeRotation().getDegrees());
        }

        // Set the yaw lock to a benign value
        yawLock = false;
        yawLockValue = swerve.getYaw();

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
        double rawLeftY = -driverController.getLeftY();
        double rawLeftX = -driverController.getLeftX();
        double rawRightX = -driverController.getRightX();
        double driveTranslateY = rawLeftY>= 0 ? (Math.pow(rawLeftY, SWERVE.JOYSTICK_EXPONENT)) : -(Math.pow(rawLeftY, SWERVE.JOYSTICK_EXPONENT));
        double driveTranslateX = rawLeftX>= 0 ? (Math.pow(rawLeftX, SWERVE.JOYSTICK_EXPONENT)) : -(Math.pow(rawLeftX, SWERVE.JOYSTICK_EXPONENT));
        double driveRotate =     rawRightX >= 0 ? (Math.pow(rawRightX, SWERVE.JOYSTICK_EXPONENT)) : -(Math.pow(rawRightX, SWERVE.JOYSTICK_EXPONENT));
        controls.update(driverController, operatorController);

        //
        // Lock the robot yaw if the rotation rate is low and the yaw joystick is released
        // Only unlock the robot yaw if the joystick provides a yaw command
        //
        if ((Math.abs(swerve.getYawRate()) < 5) && (Math.abs(rawRightX) < 0.05)) {
            if (!yawLock) {
                yawLockValue = swerve.getYaw();
            }
            yawLock = true;
        }

        if (Math.abs(rawRightX) > 0.05) {
            yawLock = false;
        }

        Logger.recordOutput(SWERVE.LOG_PATH+"YawLock", yawLock);
        Logger.recordOutput(SWERVE.LOG_PATH+"YawLast", yawLockValue);
        Logger.recordOutput(SWERVE.LOG_PATH+"YawRate", swerve.getYawRate());
        Logger.recordOutput(SWERVE.LOG_PATH+"rawRightX", rawRightX);

        //Create a new ChassisSpeeds object with X, Y, and angular velocity from controller input
        ChassisSpeeds currentSpeeds;
        currentRange = new RangeTable.RangeEntry(0, 0, 0);

        if(controls.resetGyro){
            swerve.zeroGyroscope();
            yawLock = false;
            yawLockValue = 0;
        }

        if (controls.slowMode) { //Slow Mode slows down the robot for better precision & control
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
        currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(currentSpeeds, controls.robotOriented?new Rotation2d():swerve.getGyroscopeRotation());
        noteLock.updateVision();

        if(driverController.getLeftTriggerAxis() > 0.1){
            double omega = poseVision.visual_servo(0, 10, APRILTAG_VISION.SPEAKER_AIM_TAGS, 1.5);
            currentSpeeds = new ChassisSpeeds(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, omega);
            yawLock = false;
            yawLockValue = swerve.getYaw();
        }

        if(controls.autoCollect){
            currentSpeeds = noteLock.driveToTarget(turnPID, drivePID, NOTELOCK.TELEOP_DRIVE_TO_TARGET_ANGLE);
            currentSpeeds.vxMetersPerSecond = driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo();
            currentSpeeds.vyMetersPerSecond = 0;
            controls.intake = true;
            yawLock = false;
            yawLockValue = swerve.getYaw();
        }

        if(controls.kiddyPoolShot){
            subsystemsManager.staticPrime(RangeTable.getKiddyPool());
        }
        else if(controls.shootFromPodium){
            RangeTable.RangeEntry entry = RangeTable.getPodium();
            subsystemsManager.staticPrime(entry);
        }
        else if(controls.rangeTableShoot){
            subsystemsManager.setVisionPrime();
        }
        else if (controls.trapPrime) {
            subsystemsManager.staticPrime(RangeTable.getTrap());
        }

        if(controls.passAim){
            if(currentAlliance == Alliance.Red){
                currentSpeeds.omegaRadiansPerSecond = swerve.turnToAngle(330);
                yawLockValue = 330;
            }
            if(currentAlliance == Alliance.Blue){
                currentSpeeds.omegaRadiansPerSecond = swerve.turnToAngle(30);
                yawLockValue = 30;
            }
            
            yawLock = false;
        }

        switch(driverController.getPOV()){
            case 0: case 180: // In either of these cases
                currentSpeeds.omegaRadiansPerSecond = swerve.turnToAngle(driverController.getPOV());
                yawLockValue = driverController.getPOV();
                break;
            case 90:
                currentSpeeds.omegaRadiansPerSecond = swerve.turnToAngle(270);
                yawLockValue = 270;
                break;
            case 270:
                currentSpeeds.omegaRadiansPerSecond = swerve.turnToAngle(90);
                yawLockValue = 90;
                break;
            // default:
                // PID to keep yaw locked when yaw is not commanded by the joystick
                // if (yawLock) {
                //     currentSpeeds.omegaRadiansPerSecond = swerve.turnToAngle(yawLockValue);
                // }
        }

        swerve.drive(currentSpeeds);
        leds.solidColor(LEDS.OFF);
        subsystemsManager.updateMechanismStateMachine(controls, distance, locked);
        if(controls.ledAmpSignal){
            leds.blinkColor(LEDS.YELLOW, 4);
        }
        else if(controls.partyMode){
            leds.PARTY();
        }
        leds.update(poseVision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS), 
                poseVision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)!=-1);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        leds.solidColor(LEDS.WHITE);
        leds.update(0,false);
    }

    @Override
    public void testInit() {
        SmartDashboard.putNumber("Feeder P", SHOOTER.FEEDER_MOTOR_kP);
        SmartDashboard.putNumber("Feeder I", SHOOTER.FEEDER_MOTOR_kI);
        SmartDashboard.putNumber("Feeder D", SHOOTER.FEEDER_MOTOR_kD);
        SmartDashboard.putNumber("Feeder F", SHOOTER.FEEDER_MOTOR_kF);
        SmartDashboard.putNumber("Feeder Velocity", 0);
    }

    double oldMotorkP = 0;
    double oldMotorkI = 0;
    double oldMotorkD = 0;
    double oldMotorkF = 0;
    public void testPeriodic() {
        // double shooterMotorkP = SmartDashboard.getNumber("Feeder P", 0);
        // double shooterMotorkI = SmartDashboard.getNumber("Feeder I", 0);
        // double shooterMotorkD = SmartDashboard.getNumber("Feeder D", 0);
        // double shooterMotorkF = SmartDashboard.getNumber("Feeder F", 0);
        // double shooterMotorVelocity = SmartDashboard.getNumber("Feeder Velocity", 0);
        // if(oldMotorkP != shooterMotorkP
        //         || oldMotorkI != shooterMotorkI
        //         || oldMotorkD != shooterMotorkD
        //         || oldMotorkF != shooterMotorkF){
        //     shooter.feederMotor.setPIDF(shooterMotorkP, shooterMotorkI, shooterMotorkD, shooterMotorkF, 0);
        // }
        // oldMotorkP = shooterMotorkP;
        // oldMotorkI = shooterMotorkI;
        // oldMotorkD = shooterMotorkD;
        // oldMotorkF = shooterMotorkF;
        // if(driverController.getAButton()){
        //     intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
        //     shooter.feederMotor.setVelocity(shooterMotorVelocity);
        // }
        // else if(driverController.getBButton()){
        //     shooter.setFeederVelocity(500);
        //     shooter.setTargetSpeed(1500, 1500);
        //     shooter.shoot();
        // }
        // else{
        //     intake.setIntakeVelocity(0);
        //     shooter.feederMotor.stop();
        //     shooter.stop();
        //     shooter.state = Shooter.States.NOTHING;
        // }
        // Logger.recordOutput("Feeder Speed", shooter.leftShooterMotor.getVelocity());

        

        
        if(poseVision.getTagInView() && poseVision.getCurrTagID() == 4) {
                SmartDashboard.putNumber("Tag 4 Z", poseVision.getCurrTagZ());
        }
        else if (poseVision.getTag2InView() && poseVision.getCurrTag2ID() == 4) {
            SmartDashboard.putNumber("Tag 4 Z", poseVision.getCurrTag2Z());
        }
        else {
            SmartDashboard.putNumber("Tag 4 Z", -1.0);
        }

        if (driverController.getAButton()) {
            // set elevator
            elevator.setPivotAngleCustom(SmartDashboard.getNumber("Elevator Custom Angle", 0));
            shooter.setShootVelocity((int)SmartDashboard.getNumber("Shooter Left Speed", 0), (int)SmartDashboard.getNumber("Shooter Right Speed", 0));
            Logger.recordOutput(ELEVATOR.LOG_PATH+"IsPivotReady", elevator.isTargetAngle());
            Logger.recordOutput(SHOOTER.LOG_PATH+"ShooterIsReady",shooter.readyToShoot());
            if (shooter.readyToShoot() && elevator.isTargetAngle()) {
                shooter.setFeederPower(SHOOTER.SHOOTING_FEEDER_POWER); // shoot
            }
        }
        else {
            elevator.setPivotAngleCustom(0);
            shooter.stopFlywheels(); shooter.stopFeeders();
        }

        // driverController.setRumble(RumbleType.kBothRumble, driverController.getLeftTriggerAxis()*255);

        leds.solidColor(LEDS.RED);
    } 

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}