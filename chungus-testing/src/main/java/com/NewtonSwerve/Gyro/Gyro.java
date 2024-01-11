package com.NewtonSwerve.Gyro;

public interface Gyro {
    public double getYaw();

    public double getRoll();
    
    public double getPitch();

    public void zeroYaw();

    // public boolean isRotating();

}
