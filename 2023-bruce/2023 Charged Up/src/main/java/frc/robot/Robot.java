// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LED.LEDMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.ConfigRun.AutoOptions;
import frc.robot.autonomous.AutoDrive;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.autonomous.autons.BaseAuto;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;



import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.rmi.registry.LocateRegistry;

import javax.swing.DropMode;

import com.swervedrivespecialties.swervelib.DriveController;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.LogFileUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public XboxController driverController;
  public XboxController operatorController;
  public Drivetrain drive;
  public LED ledStrips;

  public Vision turnToVision;
  public Vision driveToVision;
  public String currentPiecePipeline;
  public FRCLogger logger;
  public boolean wasZeroed = false;
  private boolean coneVision = true;
  public Power power;
  private boolean angleTapBool = false;
  public BeamSensor cubeBeamSensor;
  public SmoothingFilter smoothingFilter;



  private double currentWrist = Constants.WRIST_INTAKE_ROTATIONS;

  private BaseAuto selectedAuto;
  private AutonomousSelector selector;
  public Autopark autoPark;
  private Timer timer = new Timer();

  private boolean isPartyMode = false;

  private DriveScaler driveScaler;

  public static Field2d FIELD = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //AdvantageKit logging code
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    if (true) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
    }
    Logger.getInstance().start();
    
    
    logger = new FRCLogger(true, "CustomLogs");
    driverController = new XboxController(0);
    operatorController = new XboxController(1);
    power = new Power();
    drive = new Drivetrain(logger);
    cubeBeamSensor = new BeamSensor(Constants.BEAM_BREAK_CUBE_ID);
    turnToVision = new Vision(Constants.LIMELIGHT_VISION, Constants.DRIVE_TO_LOCK_ERROR,
     Constants.DRIVE_TO_CLOSE_ERROR, Constants.DRIVE_TO_CAMERA_HEIGHT, Constants.DRIVE_TO_CAMERA_ANGLE, 
     Constants.DRIVE_TO_TARGET_HEIGHT, logger, Constants.TURN_TO_kP, Constants.TURN_TO_kI, Constants.TURN_TO_kD);
     driveToVision = new Vision(Constants.LIMELIGHT_REAR, Constants.SUBSTATION_OFFSET,
     Constants.DRIVE_TO_CLOSE_ERROR, Constants.DRIVE_TO_CAMERA_HEIGHT, Constants.DRIVE_TO_CAMERA_ANGLE, 
     Constants.DRIVE_TO_TARGET_HEIGHT, logger, Constants.DRIVE_TO_kP, Constants.DRIVE_TO_kI, Constants.DRIVE_TO_kD);
    ledStrips = new LED(power, turnToVision);
    // intake.reset();
    // lift.reset();
    driveScaler = new DriveScaler();

    smoothingFilter = new SmoothingFilter(1, 1, 1); //5, 5, 1

    // SmartDashboard.putData(FIELD);
    selector = new AutonomousSelector();
    
    
    // SmartDashboard.putNumber("Command Counter", 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    power.powerPeriodic();
    ledStrips.updatePeriodic();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // NOTE: Removed. See FRC2023-master repo if you want to run Bruce.
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // NOTE: Removed. See FRC2023-master repo if you want to run Bruce.
  }
  
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // NOTE: Removed. See FRC2023-master repo if you want to run Bruce.
  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // NOTE: Removed. See FRC2023-master repo if you want to run Bruce.
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // NOTE: Removed. See FRC2023-master repo if you want to run Bruce.
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    ledStrips.set(LEDMode.ATTENTION);
    // SmartDashboard.putBoolean("Cube Beam Broken?", cubeBeamSensor.isBroken());


    // else if(operatorController.getBButton()){
    //     ledStrips.set(LEDMode.TARGETLOCK);
    // }
    // else if(operatorController.getXButton()){
    //     ledStrips.set(LEDMode.UP_AND_DOWN);
    // } 
    // else if (operatorController.getYButton()) {
    //   ledStrips.set(LEDMode.WAVES);
    // }
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
        0, drive.getGyroscopeRotation())); // Inverted due to Robot Directions being the
          //  intake.logBeamBreaks();

    // // opposite of controller direct
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CUBE_PIPELINE);
    
  }

  public void testPeriodic() {
    double translatePower;
    double translateX;
    double translateY;
    double rotate;
    translatePower = ConfigRun.TRANSLATE_POWER_SLOW;
    double rotatePower = ConfigRun.ROTATE_POWER_SLOW;

    ChassisSpeeds driveSpeeds = new ChassisSpeeds();

    drive.getCurrentPos();
    turnToVision.updateVision();
    driveToVision.updateVision();

    double translateXScaled = driveScaler.scale(-joystickDeadband(driverController.getLeftY()));
    double translateYScaled = driveScaler.scale(-joystickDeadband(driverController.getLeftX()));
    double rotateScaled = driveScaler.scale(joystickDeadband(driverController.getRightX())); 

    rotate = rotateScaled * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * rotatePower;
    
    translateX = translateXScaled * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * translatePower;          
    translateY = translateYScaled * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * translatePower;
    if (driverController.getBButton()) {
    
      double rotateSpeed = turnToVision.lockTargetSpeed(0, "tx", Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 0);
      rotate = rotateSpeed;

      double driveToSpeed = driveToVision.lockTargetSpeed(0, "ty", 1.0, 20); // 20 means its sorta close
      translateY = driveToSpeed; // go forwards at driveToSpeed towards the target
      SmartDashboard.putNumber("PID Forwards Vel", driveToSpeed);
    }
    ChassisSpeeds smoothedRobotRelative = smoothingFilter.smooth(new ChassisSpeeds(translateX, translateY, rotate));
    driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(      
      smoothedRobotRelative.vxMetersPerSecond, 
      smoothedRobotRelative.vyMetersPerSecond,
      rotate
    ), drive.getGyroscopeRotation());
    drive.drive(driveSpeeds);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  public double joystickDeadband(double inputJoystick) {
    if (Math.abs(inputJoystick) < ConfigRun.JOYSTICK_DEADBAND) {
      return 0;
    } else {
      return inputJoystick;
    }
  }
}
