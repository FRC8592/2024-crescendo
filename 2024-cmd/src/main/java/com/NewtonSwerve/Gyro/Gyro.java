package com.NewtonSwerve.Gyro;

public interface Gyro {
    public double getYaw();

    public double getRoll();

    public double getPitch();

    public void setYaw(double yaw);

    public void zeroYaw();

    public double getYawRate();

    // public boolean isRotating();

}
