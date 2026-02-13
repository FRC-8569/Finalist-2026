package frc.robot.Shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class Constants {
    public static final CANBus bus = new CANBus("rio");

    public static class Pitch{
        public static final int MotorID = 54;
        public static final int MotorEncoderID = 55;
        public static final double PitchGearRatio = 1;

        public static final Slot0Configs PitchPID = new Slot0Configs()
            .withKP(0).withKD(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        public static final MotionMagicConfigs PitchMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1)
            .withMotionMagicAcceleration(1);
    }

    public static class Shoot {
        public static final int MotorID = 56;
        public static final double ShootGearRatio = 1;
        public static final Slot0Configs ShootPID = new Slot0Configs()
            .withKP(0).withKD(0).withKS(12.4/125.5*Shoot.ShootGearRatio)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
        public static final MotionMagicConfigs ShootMagic = new MotionMagicConfigs()
            .withMotionMagicAcceleration(1);
    }
}
