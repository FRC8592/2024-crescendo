package frc.robot;

public final class Constants {
    private Constants(){throw new UnsupportedOperationException();}
    public static final double deadBandValue = 0.01;
    public static final double motorPowerMultiplier = 0.15;

    //constants for turret/shooter
    public static final double ballProcessSpeed = 1.0;
    public static final double ballTriggerSpeed = 0.5;
    public static final double turretLaunchSpeed = 0.5;
    public static final int turretRotationLimit = 1700;
    public static final double ticksPerDegree = 1700/90; //1700 ticks for about 90 degrees. Refine later with CAD files.
    public static final double turnToRange = 20;
    public static final double flyWheelMultiplier = 0.2;

    //////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware ID Mapping
/////////////////////////////////////////////////////////////////////////////////////////////////////




    // Joystick USB IDs
    public static final int driverStickUSB = 0;
    public static final int turretStickUSB = 1;

    // Drivetrain CAN bus IDs
    public static final int leftFrontCAN = 1;
    public static final int leftBackCAN = 3;
    public static final int rightFrontCAN = 4;
    public static final int rightBackCAN = 2;

    // Collector CAN bus IDs
    public static final int intakeSpinCAN = 16; // Spins collector wheels on intake
    public static final int compressorCAN = 20;
    public static final int intakeSolPortA = 2;
    public static final int intakeSolPortB = 3;

    // Turret CAN bus IDs
    public static final int turretLaunchCAN = 7; // High speed shooter flywheel
    public static final int turretRotateCAN = 12; // Control rotation of turret
    public static final int ballProcessCAN = 13; // Controls belt driving balls towards turret
    public static final int ballTriggerCAN = 11; // Feed balls to flywheel (5k RPM max! Be careful!)


}