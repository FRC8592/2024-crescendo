package com.NewtonSwerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.NewtonSwerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NewtonModule {
    public SwerveModule module;
    private double wheelCircumference;

    public NewtonModule(SwerveModule module, double wheelCircumference) {
        this.module = module;
        this.wheelCircumference = wheelCircumference;
    }

    public TalonFX getThrottleMotor() {
        return module.getDriveController().getDriveFalcon();
    }

    public double getThrottleEncoder() {
        return this.getThrottleMotor().getSelectedSensorPosition();
    }

    public void resetThrottleEncoder() {
        this.getThrottleMotor().setSelectedSensorPosition(0);
    }

    public void resetAbsoluteAngle() {
        this.module.getSteerController().resetAbsoluteAngle();
    }

    public void setThrottleVelocity(double inputVelocity, SwerveModule module) {
        this.getThrottleMotor().set(ControlMode.Velocity, inputVelocity);
    }

    public void setModule(double steerAngle, double velocityMetersPerSecond) {
        double velocityToApply;
        if (module.setModuleSteerAngle(steerAngle)) {
            velocityToApply = -velocityMetersPerSecond;
        } else {
            velocityToApply = velocityMetersPerSecond;
        }
        setThrottleVelocity(velocityToApply, module);
    }

    public void setThrottleCurrentLimit(double currentLimit) {
        this.getThrottleMotor().configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, 0, 0));
        this.getThrottleMotor().configClosedloopRamp(0.02);
    }

    public double getAppliedCurrent() {
        return this.getThrottleMotor().getStatorCurrent();
    }

    public double getThrottleVelocity(SwerveModule module) {
        return this.getThrottleMotor().getSelectedSensorVelocity();
    }

    public double getSteerAngle() {
        return this.module.getSteerAngle();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(this.getThrottleEncoder() / 4096.0 / this.wheelCircumference,
                new Rotation2d(this.getSteerAngle()));
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(this.module.getDriveVelocity(), Rotation2d.fromRadians(this.getSteerAngle()));
    }
}
