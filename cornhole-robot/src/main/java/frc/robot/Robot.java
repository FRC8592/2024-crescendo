// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.wpilog.*;
import org.littletonrobotics.junction.networktables.*;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;
  private DriveTrain drive;
  private XboxController gamePad;
  // private Turret turret;
  private Launcher launcher;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //AdvantageKit init code
    Logger.getInstance().recordMetadata("ProjectName", "MyProject");
    if (isReal()) {
      Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // TODO: This might need to change depending on where the USB stick mounts.
      Logger.getInstance().addDataReceiver(new NT4Publisher());
    }
    else {
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
      Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
    Logger.getInstance().start();


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();
    drive    = new DriveTrain();
    gamePad  = new XboxController(Constants.DRIVER_GAMEPAD_PORT);
    //turret   = new Turret();
    launcher = new Launcher();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //if (m_autonomousCommand != null) {
    //  m_autonomousCommand.cancel();
    //}

    // Ensure the launch motor is not moving
    launcher.setIdle();
    launcher.reZero();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Variables for input from the gamePad
    //double leftStickY = gamePad.getLeftY();
    //double leftStickX = gamePad.getLeftX();
    //double rightStickX = gamePad.getRightX();

    //drive.xyDrive(leftStickX, leftStickY);
    //turret.speedRotate(rightStickX*0.25);



    //Update the logging and state machine on the launcher
    launcher.update();

    // Hold the left bumper and click the right bumper to launch. Given
    // that we need people to walk up to this thing to load the beanbag,
    // it's probably best to make it so a misclick or dropping the
    // controller isn't likely to cause it to launch. Especially not at
    // a public demo.

    if (gamePad.getRightBumperPressed() && gamePad.getLeftBumper()){
      launcher.launch();
    }

    //There should be no need for this, but just in case...
    if(gamePad.getAButton()){
      launcher.reZero();
    }
    if(gamePad.getBButton()){
      drive.xyDrive((gamePad.getRightTriggerAxis()-gamePad.getLeftTriggerAxis())/3, 0);
      launcher.up();
    }
    if(gamePad.getBButtonReleased()){
      launcher.setIdle();
    }
    if(gamePad.getYButton()){
      drive.xyDrive(gamePad.getLeftX(), gamePad.getLeftY());
      launcher.up();
    }
    if(gamePad.getYButtonReleased()){
      launcher.setIdle();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
