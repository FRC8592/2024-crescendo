package com.NewtonSwerve.Gyro;

import com.kauailabs.navx.frc.AHRS;

public class NewtonNavX implements Gyro {

    private AHRS navX;

    public NewtonNavX(AHRS navX) {
        this.navX = navX;
    }

    public double getYaw() {
        return this.navX.getYaw();
    }

    public double getRoll() {
        return this.navX.getRoll();
    }

    public double getPitch() {
        return this.navX.getPitch();
    }

    public void zeroYaw() {
        this.navX.zeroYaw();
    }
    public void setYaw(double yaw){
        System.err.println("Can't set yaw on a NavX");
    }
}
