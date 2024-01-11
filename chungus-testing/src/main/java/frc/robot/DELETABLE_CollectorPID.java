// package frc.robot;


// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;

// // import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// public class CollectorPID {
//   private double kP;
//   private double kI;
//   private double kD;
//   private double kF;
//   private double collectorSpeed;
//   private WPI_TalonFX collectorMotor;

//   public CollectorPID(double kP, double kI, double kD, double kF){
//     this.kP = kP;
//     this.kI = kI;
//     this.kD = kD;
//     this.kF = kF;
//     collectorSpeed = 0;

//     //initialize and configure motors
//     collectorMotor = new WPI_TalonFX(Constants.INTAKE_ROLLER_MOTOR_CAN_ID);
//     collectorMotor.configFactoryDefault();
//     collectorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
//     collectorMotor.setInverted(true);
//     collectorMotor.setNeutralMode(NeutralMode.Brake);
    


//     //set PID values
//     collectorMotor.config_kP(0, kP);
//     collectorMotor.config_kI(0, kI);
//     collectorMotor.config_kD(0, kD);
//     collectorMotor.config_kF(0, kF);
//     collectorMotor.selectProfileSlot(0, 0);
//     //make sure motor is at 0 speed
//     collectorMotor.set(ControlMode.Velocity, collectorSpeed);
//   }  

//   public void updateVelocity(double targetSpeed, int mode){
//     if(targetSpeed == 0) {
//       collectorMotor.set(ControlMode.PercentOutput, 0);
//     }
//     else {
//       double targetSpeedTicks = Constants.CONVERSION_RPM_TO_TICKS_MS * targetSpeed;
//       collectorMotor.set(ControlMode.Velocity, targetSpeedTicks);
//     }

//     SmartDashboard.putNumber("Collector Rotation Expected Speed", targetSpeed);
//     //SmartDashboard.putNumber("Collector Rotation Actual Speed", this.getVelocity());
//     SmartDashboard.putNumber("Collector Rotation Error", this.getVelocity() - targetSpeed);
//   }

//   public double getVelocity(){
//     double currentSpeedRPM = collectorMotor.getSelectedSensorVelocity() / Constants.CONVERSION_RPM_TO_TICKS_MS;

//     return currentSpeedRPM;
//   }

// }