package frc.robot;

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
    private LimelightTargeting apriltagVision;
    // private PoseVision apriltagLockYaw;
    // private PoseVision apriltagLockY;
    // private PoseVision poseGetter;
    private NeoPixelLED leds;
    private Power power;
    private PIDController turnPID;
    private PIDController drivePID;
    private SmoothingFilter smoothingFilter;

    private ButtonManager slowMode = new ButtonManager(false);
    private ButtonManager resetGyro = new ButtonManager(false);
    private ButtonManager autoCollect = new ButtonManager(false);
    private ButtonManager robotOriented = new ButtonManager(false);

    // operator controls
    private ButtonManager ledAmpSignal = new ButtonManager(false);
    private ButtonManager shootFromPodium = new ButtonManager(false);
    private ButtonManager outake = new ButtonManager(false);
    private ButtonManager intaking = new ButtonManager(false);
    private ButtonManager stow = new ButtonManager(false);
    private ButtonManager amp = new ButtonManager(false);
    private ButtonManager climb = new ButtonManager(false);
    private ButtonManager speakerAmp = new ButtonManager(false);
    private ButtonManager manualRaiseClimber = new ButtonManager(false);
    private ButtonManager manualLowerClimber = new ButtonManager(false);

    private enum States{
        //-1 means the code in the case will handle it

        // Variables:
        //        intake roller speed,    elevator pivot angle,        elevator extension length,        shooter feeder speed,           shooter flywheel speed
        STOWED   (0,                      ELEVATOR.PIVOT_ANGLE_STOWED, ELEVATOR.EXTENSION_LENGTH_STOWED, 0,                              0                                ),
        INTAKING (-1,                     ELEVATOR.PIVOT_ANGLE_STOWED, ELEVATOR.EXTENSION_LENGTH_STOWED, SHOOTER.INTAKE_FEEDER_VELOCITY, SHOOTER.INTAKE_FLYWHEEL_VELOCITY ),
        OUTAKING (INTAKE.OUTAKE_VELOCITY, -1,                          -1,                               SHOOTER.OUTAKE_FEEDER_VELOCITY, SHOOTER.OUTAKE_FLYWHEEL_VELOCITY ),
        AMP      (0,                      ELEVATOR.PIVOT_ANGLE_AMP,    ELEVATOR.EXTENSION_LENGTH_AMP,    -1,                             -1                               ),
        CLIMB    (0,                      ELEVATOR.PIVOT_ANGLE_CLIMB,  -1,                               0,                              0                                ),
        SHOOT    (0,                      -1,                          0,                                -1,                             -1                               ),
        ;

        public double intakeRollerSpeed;
        public double elevatorPivotAngle;
        public double elevatorExtensionLength;
        public double shooterFeederSpeed;
        public double shooterFlywheelSpeed;

        States(double intakeRollerSpeed, double elevatorPivotAngle, double elevatorExtensionLength, double shooterFeederSpeed, double shooterFlywheelSpeed){
            this.intakeRollerSpeed = intakeRollerSpeed;
            this.elevatorPivotAngle = elevatorPivotAngle;
            this.elevatorExtensionLength = elevatorExtensionLength;
            this.shooterFeederSpeed = shooterFeederSpeed;
            this.shooterFlywheelSpeed = shooterFlywheelSpeed;
        }
    }

    private States state;

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
        
        state = States.STOWED;
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

        Logger.recordOutput(SHOOTER.LOG_PATH+"FeederSpeedRPM", shooter.feederMotor.getVelocity());
        Logger.recordOutput(INTAKE.LOG_PATH+"IntakeVelocityRPM", intake.getIntakeVelocity());

        //SmartDashboard.putNumber("target angle", targetAngle);
        elevator.update();
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
        double driveTranslateY = rawLeftY; //>= 0 ? (Math.pow(rawLeftY, SWERVE.JOYSTICK_EXPONENT)) : -(Math.pow(rawLeftY, SWERVE.JOYSTICK_EXPONENT));
        double driveTranslateX = rawLeftX; //>= 0 ? (Math.pow(rawLeftX, SWERVE.JOYSTICK_EXPONENT)) : -(Math.pow(rawLeftX, SWERVE.JOYSTICK_EXPONENT));
        double driveRotate =     rawRightX >= 0 ? (Math.pow(rawRightX, SWERVE.JOYSTICK_EXPONENT)) : -(Math.pow(rawRightX, SWERVE.JOYSTICK_EXPONENT));
        slowMode.update          (driverController.getRightBumper());
        resetGyro.update         (driverController.getBackButton());
        autoCollect.update       (driverController.getLeftBumper());
        robotOriented.update     (driverController.getRightTriggerAxis() >0.1);

        //operator controls
        // shooter/feeder functions
        shootFromPodium.update   (operatorController.getLeftBumper());

        outake.update            (operatorController.getRightBumper());
        intaking.update          (operatorController.getLeftTriggerAxis()>0.1);

        stow.update              (operatorController.getAButton());
        amp.update               (operatorController.getXButton());
        climb.update             (operatorController.getYButton());
        speakerAmp.update        (operatorController.getRightTriggerAxis()>0.1);
        manualRaiseClimber.update(operatorController.getPOV() == 0);
        manualLowerClimber.update(operatorController.getPOV() == 180);
        ledAmpSignal.update      (operatorController.getBackButton());

        //Create a new ChassisSpeeds object with X, Y, and angular velocity from controller input
        ChassisSpeeds currentSpeeds;
        if(resetGyro.isPressed()){
            swerve.zeroGyroscope();
        }

        if (slowMode.isPressed()) { //Slow Mode slows down the robot for better precision & control
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
        currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(currentSpeeds, robotOriented.isPressed() ? new Rotation2d() : swerve.getGyroscopeRotation());
        noteLock.updateVision();

        if(autoCollect.isPressed()){
            currentSpeeds = noteLock.driveToTarget(turnPID, drivePID, NOTELOCK.TELEOP_DRIVE_TO_TARGET_ANGLE);
            state = States.INTAKING;
        }
        else if(intaking.isRisingEdge()){
            state = States.INTAKING;
        }
        else if(outake.isRisingEdge()){
            state = States.OUTAKING;
        }
        else if(amp.isTriggerRisingEdge()){
            state = States.AMP;
            climb.resetTrigger();
        }
        else if(climb.isTriggerRisingEdge()){
            state = States.CLIMB;
            amp.resetTrigger();
        }
        else if((intaking.isFallingEdge() && state == States.INTAKING)
                || (outake.isFallingEdge() && state == States.OUTAKING)
                || stow.isPressed()){
            state = States.STOWED;
        }

        // Set all the defaults. `-1`s are written in the switch statement
        if(state.intakeRollerSpeed != -1){intake.setIntakeVelocity(state.intakeRollerSpeed);}
        if(state.elevatorPivotAngle != -1){elevator.setPivotAngle(state.elevatorPivotAngle);}
        if(state.elevatorExtensionLength != -1){elevator.setExtensionLength(state.elevatorExtensionLength);}
        if(state.shooterFeederSpeed != -1){shooter.setFeederVelocity(state.shooterFeederSpeed);}
        if(state.shooterFlywheelSpeed != -1){shooter.setFlywheelVelocity(state.shooterFlywheelSpeed);}
        switch (state) {
            case STOWED: //NOT SET: Nothing
                if(speakerAmp.isRisingEdge() || shootFromPodium.isRisingEdge()){
                    state = States.SHOOT;
                }
                break;

            case SHOOT: //NOT SET: Elevator pivot angle, shooter feeder speed, shooter flywheel speed
                if(speakerAmp.isPressed()){
                    shooter.setFlywheelVelocity(RangeTable.get(0).flywheelSpeed);
                    elevator.setPivotAngle(RangeTable.get(0).pivotAngle);
                }
                else if(shootFromPodium.isPressed()){
                    shooter.setFlywheelVelocity(RangeTable.get(1).flywheelSpeed);
                    elevator.setPivotAngle(RangeTable.get(1).pivotAngle);
                }
                else{
                    state = States.STOWED;
                    break; // Don't run anything after this
                }

                if(shooter.isReady() && elevator.isTargetPivotAngle()){
                    shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_VELOCITY);
                }
                break;

            case INTAKING: //NOT SET: Intake roller speed
                if(elevator.isTargetPivotAngle()){
                    intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
                }
                else{
                    intake.setIntakeVelocity(0);
                }
                // Possibly add code to make it automatically stop intaking when we get a note.
                break;

            case OUTAKING: //NOT SET: Elevator pivot angle, elevator extension length
                // Freeze the elevator
                elevator.setPivotAngle(elevator.getPivotAngle());
                elevator.setExtensionLength(elevator.getExtensionLength());
                break;

            case AMP: //NOT SET: Shooter feeder speed, shooter flywheel speed
                if(speakerAmp.isPressed()){
                    shooter.setFeederVelocity(SHOOTER.AMP_FEEDER_VELOCITY);
                    shooter.setFlywheelVelocity(SHOOTER.AMP_FLYWHEEL_VELOCITY);
                }
                else{
                    shooter.setFeederVelocity(0);
                    shooter.setFlywheelVelocity(0);
                }
                break;

            case CLIMB: //NOT SET: Elevator extension length
                if(climb.isRisingEdge()){ // If this is the first frame that we're climbing
                    elevator.setExtensionLength(ELEVATOR.EXTENSION_LENGTH_CLIMB);
                }
                if(manualRaiseClimber.isPressed()){
                    elevator.extend();
                }
                if(manualLowerClimber.isPressed()){
                    elevator.retract();
                }
                break;
        }

        //LED management
        if (ledAmpSignal.isRisingEdge()) {
            leds.flashTimer.reset();
        }
        if (ledAmpSignal.isPressed()) {
            leds.amp();
        }
        else if (shooter.hasNote) {
            leds.notePickup();
        } else {
            leds.off();
        }
        swerve.drive(currentSpeeds);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        leds.disabled();
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {
        // driverController.setRumble(RumbleType.kBothRumble, driverController.getLeftTriggerAxis() * 255);
        // operatorController.setRumble(RumbleType.kBothRumble, operatorController.getLeftTriggerAxis() * 255);
        SmartDashboard.putNumber("Robot Yaw", swerve.getGyroscopeRotation().getDegrees());
    } 

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}