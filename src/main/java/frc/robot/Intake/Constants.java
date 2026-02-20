package frc.robot.Intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.GlobalConstants;

public class Constants {
    public static final CANBus bus = new CANBus("rio");
    public class Tongue {
        public static final int MotorID = 51;
        public static final double GearRatio = 20/1.5;
        public static final Slot0Configs TonguePID = new Slot0Configs()
            .withKP(7).withKD(0.3)
            .withKS(0).withKV(0).withKA(0);
        public static final MotionMagicConfigs TongueMagic = new MotionMagicConfigs()
            .withMotionMagicExpo_kV(Volts.of(12.4/GlobalConstants.KrakenX60FOCSpeed*GearRatio).per(RotationsPerSecond))
            .withMotionMagicExpo_kA(Volts.of(12.4/80).per(RotationsPerSecondPerSecond));
        public static final CurrentLimitsConfigs TongueCurrentLimit = new CurrentLimitsConfigs();
    }

    public class Roll {
        public static final int MotorID = 52;
        public static final double GearRatio = 32.0/15;
        public static final Slot0Configs RollPID = new Slot0Configs()
            .withKP(0).withKD(0).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
        public static final MotionMagicConfigs RollMagic = new MotionMagicConfigs()
            .withMotionMagicExpo_kV(Volt.of(12.4/(GlobalConstants.KrakenX44FOCSpeed*GearRatio)).per(RotationsPerSecond))
            .withMotionMagicExpo_kA(Volt.of(12.4/(80)).per(RotationsPerSecondPerSecond));
        public static final CurrentLimitsConfigs RollCurrentLimit = new CurrentLimitsConfigs();
    }
}
