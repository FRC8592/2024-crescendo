package frc.robot;

public final class Constants {
    public final class SHARED {
        public static final String LOG_FOLDER = "CustomLogs";
    }

    public final class CONTROLLERS {
        public static final int CONTROLLER_PORT = 0;
    }

    public final class CAN {
        public static final int MOTOR_CAN_ID = 30;
        public static final int PDH_CAN_ID = 1;
    }

    public final class SINGLEMOTOR{
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"SingleMotor";

        // TODO: set these PID and sim constants once SysID has done its thing
        public static final double PID_P = 0.079922;
        public static final double PID_I = 0;
        public static final double PID_D = 0;

        public static final double FF_S = 0;
        public static final double FF_V = 0.10844;
        public static final double FF_A = 0.0039994;
    }
}
