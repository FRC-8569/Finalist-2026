package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static final CANBus bus = new CANBus();
    public class Pitching {
        public static final int MotorID = 54;
        public static final int EncoderID = 56;  
        public static final double PitchingGearRatio = 1;

        public static final Slot0Configs PitchPID = new Slot0Configs()
            .withKP(0).withKD(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        public static final MotionMagicConfigs PitchingMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1));
    }

    public class Rolling {
        public static final int MotorID = 55;
        public static final double RollingGearRatio = 1;
        public static final Distance WheelCirc = Inches.of(2.25).times(Math.PI);
        public static final Slot0Configs RollingPID = new Slot0Configs()
            .withKP(0).withKD(0).withKV(12.4/100*RollingGearRatio);
        public static final MotionMagicConfigs RollingMagic = new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1));
    }

    public class index {
        public static final int MotorID = 56;
        public static final double IndexGearRatio = 1;
    }
}
