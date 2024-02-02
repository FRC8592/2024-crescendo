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
        pigeon = new NewtonPigeon(new Pigeon2(PIGEON.CAN_ID));
        swerve = new Swerve(pigeon);
        power = new Power();
        leds = new LED();
        shooter = new Shooter();
        poseGetter = new PoseVision();
        intake = new Intake();
        
        noteLock = new LimelightTargeting(NOTELOCK.LIMELIGHT_NAME, NOTELOCK.LOCK_ERROR, NOTELOCK.CAMERA_HEIGHT,
                NOTELOCK.kP, NOTELOCK.kI, NOTELOCK.kD);
        elevator = new Elevator();
        
    }

    @Override
    public void robotPeriodic() {
        

    }

    @Override
    public void autonomousInit() {
        shooter.setAlliance(DriverStation.getAlliance().get());
        currentAuto = autoSelect.getSelectedAutonomous();
        currentAuto.addModules(swerve, null /* vision */);
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
        shooter.setAlliance(DriverStation.getAlliance().get());
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setTeleopCurrentLimit();
    }

    @Override
    public void teleopPeriodic() {
        //TODO: Move the auto-aim functions to driver controller, make sure no others are needed, and add in the logic for what they will do

        /* Driver Controls:
         * translate = left stick
         * rotate = right stick x
         * slowmode = right bumper
         * 
         * OPERATOR CONTROLLER:
         * 
         * 
         */
        double driveTranslateY = driverController.getLeftY();
        double driveTranslateX = driverController.getLeftX();
        double driveRotate = driverController.getRightX();
        boolean slowMode = driverController.getRightBumper();
       
        boolean intaking = operatorController.getAButton();
        boolean autoIntake = operatorController.getLeftTriggerAxis() > 0.1;
        boolean shooting = operatorController.getBButton();
        boolean autoShoot = operatorController.getRightTriggerAxis() > 0.1;
        boolean ampPosition = operatorController.getXButton();
        boolean ampScore = operatorController.getRightBumper();
        boolean hangToggle = operatorController.getYButtonPressed();
        
        
        
        
        ChassisSpeeds currentSpeeds;
        if (slowMode) {
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
        shooter.setAlliance(DriverStation.getAlliance().get());
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}