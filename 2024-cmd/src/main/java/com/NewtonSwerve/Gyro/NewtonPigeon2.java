package com.NewtonSwerve.Gyro;

import com.ctre.phoenix.sensors.Pigeon2;

public class NewtonPigeon2 implements Gyro {
    private double lastYawSample = 0.0;

    private Pigeon2 pigeon;

    public NewtonPigeon2(Pigeon2 pigeon) {
        this.pigeon = pigeon;
        this.pigeon.configFactoryDefault();
    }

    public double getYaw() {
        return this.pigeon.getYaw() % 360.0;
    }

    public double getRoll() {
        return this.pigeon.getRoll();
    }

    public double getPitch() {
        return this.pigeon.getPitch();
    }

    public void zeroYaw() {
        this.pigeon.setYaw(0);
    }
    public void setYaw(double yaw){
        this.pigeon.setYaw(yaw);
    }

    public double getYawRate() {
        double currentYawSample = this.getYaw();
        double yawRate = Math.abs(lastYawSample - currentYawSample) / 0.02;
        lastYawSample = currentYawSample;
        return yawRate;
    }


}
