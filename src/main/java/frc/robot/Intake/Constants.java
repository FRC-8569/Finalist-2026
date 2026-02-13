package frc.robot.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static final CANBus bus = new CANBus();
    public class Tongue {
        public static final int MotorID = 51;
        public static final int EncdoerID = 53;
        public static final Slot0Configs TonguePID = new Slot0Configs()
            .withKP(7).withKD(0.3)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        
        public static final MotionMagicConfigs TongueMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(15)
            .withMotionMagicAcceleration(30);

        public static final double GearRatio = 20*1/1.5;
        public static final Distance tolerance = Centimeters.of(1);
        public static final Current StallLimit = Amps.of(9);
    }

    public class Roller {
        public static final int MotorID = 52;
        public static final Slot0Configs RollerPID = new Slot0Configs()
            .withKP(0.7).withKV(12.4/125.5*Roller.GearRatio)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        public static final MotionMagicConfigs RollerMagic = new MotionMagicConfigs()
            .withMotionMagicAcceleration(100);
        public static final double GearRatio = 32.0/15;
        public static final Distance WheelCirc = Inches.of(2.25).times(Math.PI);
        public static final LinearVelocity[] SpeedRange = {MetersPerSecond.of(5), MetersPerSecond.of(10.5)};
    }

}