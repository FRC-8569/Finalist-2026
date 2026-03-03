package frc.robot.Spindexer;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public class Index{
        public static final int MotorID = 56;
        public static final CANBus bus = new CANBus("Drivetrain");
    }

    public class Spindex {
        public static final int MotorID = 57;
        public static final CANBus bus = new CANBus("rio");
    }
}
