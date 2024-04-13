package com.NewtonSwerve.Gyro;

import com.ctre.phoenix.sensors.PigeonIMU;

public class NewtonPigeon implements Gyro {
    /**
     * For Pigeon 1 instead of Pigeon 2
     */

    private double lastYawSample = 0.0;

    private PigeonIMU pigeon;

    public NewtonPigeon(PigeonIMU pigeon) {
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
