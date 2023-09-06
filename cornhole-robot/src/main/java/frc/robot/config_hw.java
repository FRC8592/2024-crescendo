//////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware ID Mapping
/////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

public final class config_hw {
    private config_hw(){throw new UnsupportedOperationException();}

    // Joystick USB IDs
    public static final int driverStickUSB = 0;
    public static final int turretStickUSB = 1;

    // Drivetrain CAN bus IDs
    public static final int leftFrontCAN  = 1;
    public static final int leftBackCAN   = 3;
    public static final int rightFrontCAN = 4;
    public static final int rightBackCAN  = 2;

    // Collector CAN bus IDs
    public static final int intakeSpinCAN = 16;     // Spins collector wheels on intake
    public static final int compressorCAN = 20;
    public static final int intakeSolPortA = 2;
    public static final int intakeSolPortB = 3;

    // Turret CAN bus IDs
    public static final int turretLaunchCAN = 7;      // High speed shooter flywheel
    public static final int turretRotateCAN = 12;     // Control rotation of turret
    public static final int ballProcessCAN  = 13;     // Controls belt driving balls towards turret
    public static final int ballTriggerCAN  = 11;     // Feed balls to flywheel (5k RPM max!  Be careful!)
}