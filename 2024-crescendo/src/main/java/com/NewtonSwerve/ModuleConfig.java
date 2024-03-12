package com.NewtonSwerve;

public abstract class ModuleConfig {
    protected double nominalVoltage = 12.0;

    protected double maxVelocityMetersPerSecond = 4.5;

    // throttle motor PID values
    public double throttlekP = Double.NaN;
    public double throttlekI = Double.NaN;
    public double throttlekD = Double.NaN;

    // steer motor PID values
    public double steerkP = Double.NaN;
    public double steerkI = Double.NaN;
    public double steerkD = Double.NaN;

    private double DRIVETRAIN_WIDTH_METERS; // x axis
    private double DRIVETRAIN_LENGTH_METERS; // y axis

    private double wheelCircumference;

    private final double falconTicksToMeters = 1.0 / 4096.0 / this.wheelCircumference;

    public ModuleConfig() {
    }

    public double getDriveTrainWidthMeters() {
        return this.DRIVETRAIN_WIDTH_METERS;
    }

    public void setDriveTrainWidthMeters(double driveTrainWidthMeters) {
        this.DRIVETRAIN_WIDTH_METERS = driveTrainWidthMeters;
    }

    public double getDriveTrainLengthMeters() {
        return this.DRIVETRAIN_LENGTH_METERS;
    }

    public void setDriveTrainLengthMeters(double driveTrainLengthMeters) {
        this.DRIVETRAIN_LENGTH_METERS = driveTrainLengthMeters;
    }

    public double getWheelCircumference() {
        return this.wheelCircumference;
    }

    public void setWheelCircumference(double wheelCircumference) {
        this.wheelCircumference = wheelCircumference;
    }

    public double getFalconTicksToMeters() {
        return this.falconTicksToMeters;
    }

    /**
     * 
     * @param kP The proportional value to set to the throttle motor
     * @param kI The integral value to set to the throttle motor
     * @param kD The derivative value to set to the throttle motor
     */
    public void setThrottlePID(double kP, double kI, double kD) {
        this.throttlekP = kP;
        this.throttlekI = kI;
        this.throttlekD = kD;
    }

    /**
     * 
     * @param kP The proportional value to set to the steer motor
     * @param kI The integral value to set to the steer motor
     * @param kD The derivative value to set to the steer motor
     */
    public void setSteerPID(double kP, double kI, double kD) {
        this.steerkP = kP;
        this.steerkI = kI;
        this.steerkD = kD;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public double getMaxVelocityMetersPerSecond() {
        return this.maxVelocityMetersPerSecond;
    }

    public void setMaxVelocityMetersPerSecond(double maxVelocityMetersPerSecond) {
        this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
    }
}
