package frc.robot.Intake;

import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static final CANBus bus = new CANBus("Drivetrain");
    public class Tongue {
        public static final int MotorID = 51;
        public static final int EncdoerID = 53;
        public static final Slot0Configs TonguePID = new Slot0Configs()
            .withKP(0).withKD(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        
        public static final MotionMagicConfigs TongueMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1)
            .withMotionMagicAcceleration(1);

        public static final double GearRatio = 1;
        public static final Distance PitchDiameter = Millimeters.of(60);
    }

    public class Roller {
        public static final int MotorID = 52;
        public static final Slot0Configs RollerPID = new Slot0Configs()
            .withKP(0).withKV(12.4/100*Roller.GearRatio)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        public static final MotionMagicConfigs RollerMagic = new MotionMagicConfigs()
            .withMotionMagicAcceleration(1);
        public static final double GearRatio = 1;
    }

}