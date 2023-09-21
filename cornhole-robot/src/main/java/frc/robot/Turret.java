// package frc.robot;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import edu.wpi.first.math.controller.PIDController;

// public class Turret {

//   // Create the motor objects
//   private WPI_TalonFX turretRotate;
//   private WPI_TalonFX turretLaunch;
//   private PIDController launchPID;


//   public Turret() {
//     // Define & configure the turret rotation motor
//     turretRotate = new WPI_TalonFX(Constants.TURRET_ROTATE_CAN_ID);
//     turretRotate.configFactoryDefault();
//     turretRotate.setNeutralMode(NeutralMode.Brake);
//     turretRotate.setSelectedSensorPosition(0);

//     turretLaunch = new WPI_TalonFX(Constants.TURRET_LAUNCH_CAN_ID);
//     turretLaunch.configFactoryDefault();
//     turretLaunch.setInverted(true);

//     launchPID =
//         new PIDController(Constants.FOUR_BAR_kP, Constants.FOUR_BAR_kI, Constants.FOUR_BAR_kD);


//   }

//   public void launchFourBar() {
//     if (this.isFourBarStowed()) {
//       turretLaunch.set(ControlMode.Velocity, launchPID.calculate(this.getLaunchMotorPos(),
//           Constants.FOURBAR_LAUNCH_MAX_ROTATION_TICKS));
//     } else {
//       this.resetFourBar();
//     }
//   }

//   public void resetFourBar() {
//     if (!this.isFourBarStowed()) {
//       turretLaunch.set(ControlMode.PercentOutput, 0.1);
//     }
//     turretLaunch.set(ControlMode.PercentOutput, 0);

//   }

//   public boolean isFourBarStowed() {
//     double curPos = this.getLaunchMotorPos();

//     return curPos >= -1 && curPos <= 5;
//   }

//   // Our old P-Controller turnTo function
//   public void turnTo(double targetDegrees) {
//     double encoderValue = getRotMotorPos();
//     double targetTicks = (degreesToTicks(targetDegrees)); // Convert the degrees input to ticks
//     double speed = (targetTicks - encoderValue) / 200; // How far the motor must turn to reach its
//                                                        // destination, divided by 200
//     if (speed > 1) { // Limit
//       speed = 1.0; // the
//     } // motor
//     if (speed < (-1)) { // speed
//       speed = -1.0; // to
//     } // 1.0
//     speedRotate(speed);
//   }


//   // Rotate based on a speed input
//   public void speedRotate(double speed) {
//     double encoderValue = getRotMotorPos();
//     if ((Math.abs(encoderValue) < Constants.TURRET_ROTATION_LIMIT) || // IF our encoder reads that
//                                                                       // we're within our limit OR
//         (encoderValue > Constants.TURRET_ROTATION_LIMIT && speed < 0) || // We're greater than our
//                                                                          // limit but the user wants
//                                                                          // to move backward OR
//         (encoderValue < -Constants.TURRET_ROTATION_LIMIT && speed > 0)) { // We're lower than the
//                                                                           // negative of our limit
//                                                                           // but the user wants to
//                                                                           // move forward, THEN:
//       turretRotate.set(ControlMode.PercentOutput, deadBand(speed)); // Allow the turret to rotate as
//                                                                     // the user asks
//     } else { // OTHERWISE (the user wants to do something dangerous):
//       stopRotation(); // Lock the motor
//     }
//   }

//   // This is mostly just a method for Robot.java to be able to stop the rotation motor
//   public void stopRotation() {
//     turretRotate.set(ControlMode.PercentOutput, 0);
//   }


//   public double getRotMotorPos() {
//     return turretRotate.getSelectedSensorPosition();
//   }

//   public double getLaunchMotorPos() {
//     return turretLaunch.getSelectedSensorPosition();
//   }



//   // Convert from degrees to ticks; used in turnTo, where we input degrees
//   public double degreesToTicks(double degrees) {
//     return degrees * Constants.TICKS_PER_DEGREE;
//   }

//   // Deadband method for the motors
//   public double deadBand(double value) {
//     if (Math.abs(value) <= Constants.deadBandValue) {
//       return 0;
//     } else {
//       return value;
//     }
//   }
// }
