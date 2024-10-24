package com.NewtonSwerve.Gyro;

import com.ctre.phoenix.sensors.Pigeon2;

public class NewtonPigeon implements Gyro {

    private Pigeon2 pigeon;

    public NewtonPigeon(Pigeon2 pigeon) {
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
    
}
