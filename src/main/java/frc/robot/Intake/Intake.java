package frc.robot.Intake;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import frc.robot.Intake.Constants.Tongue;
import frc.robot.Intake.Constants.Roller;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    public TalonFX TongueMotor, RollerMotor;
    public CANcoder TongueEncoder;
    private static Intake inst;

    private TalonFXConfiguration TongueConfig, RollerConfig;
    private CANcoderConfiguration TongueEncoderConfig;

    public MotionMagicTorqueCurrentFOC LickingPID;
    public MotionMagicVelocityTorqueCurrentFOC RollingPID;

    private Intake(){
        TongueMotor = new TalonFX(Tongue.MotorID, Constants.bus);
        RollerMotor = new TalonFX(Roller.MotorID, Constants.bus);
        TongueEncoder = new CANcoder(Tongue.EncdoerID, Constants.bus);

        TongueConfig = new TalonFXConfiguration();
        RollerConfig = new TalonFXConfiguration();
        TongueEncoderConfig = new CANcoderConfiguration();

        LickingPID = new MotionMagicTorqueCurrentFOC(0);
        RollingPID = new MotionMagicVelocityTorqueCurrentFOC(0);

        // TODO: Software limit TBD
        TongueConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        TongueConfig.Feedback
            .withRotorToSensorRatio(Tongue.GearRatio)
            .withFusedCANcoder(TongueEncoder);
        TongueConfig.withSlot0(Tongue.TonguePID);
        TongueConfig.withMotionMagic(Tongue.TongueMagic);

        RollerConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        RollerConfig.Feedback
            .withSensorToMechanismRatio(Roller.GearRatio);
        RollerConfig.withSlot0(Roller.RollerPID);
        RollerConfig.withMotionMagic(Roller.RollerMagic);

        TongueEncoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        TongueMotor.getConfigurator().apply(TongueConfig);
        RollerMotor.getConfigurator().apply(RollerConfig);
        TongueEncoder.getConfigurator().apply(TongueEncoderConfig);
    }

    public Distance getIntakePositionPresice(){
        return Centimeters.of(TongueMotor.getPosition().getValue().in(Rotations)*Tongue.PitchDiameter.in(Centimeters));
    }

    public IntakePosition getInsIntakePosition(){
        if(IntakePosition.Idle.getValue().minus(getIntakePositionPresice()).abs(Millimeters) < 5) return IntakePosition.Idle;
        else if(IntakePosition.Out.getValue().minus(getIntakePositionPresice()).abs(Millimeters) < 5) return IntakePosition.Idle;
        else return null;
    }

    public Command moveIntake(IntakePosition position){
        return run(() -> TongueMotor.setControl(LickingPID.withPosition(Rotations.of(position.getValue().div(Tongue.PitchDiameter).in(Percent))))).withName("Moving...");
    }

    public Command runIntake(){
        return runEnd(
            () -> RollerMotor.setControl(RollingPID.withVelocity(RotationsPerSecond.of(1))), 
            () -> RollerMotor.setControl(RollingPID.withVelocity(RotationsPerSecond.of(0)))).withName("Rolling...");
    }

    public Command runIntake(BooleanSupplier endCond){
        return runEnd(
            () -> RollerMotor.setControl(RollingPID.withVelocity(RotationsPerSecond.of(1))), 
            () -> RollerMotor.setControl(RollingPID.withVelocity(RotationsPerSecond.of(0)))).until(endCond);
    }

    @Override
    public void periodic(){
        DogLog.log("Intake/Position", getInsIntakePosition().toString());
        DogLog.log("Intake/Debug/IntakePositon", getIntakePositionPresice());
        DogLog.log("Intake/CurrentDoing", getCurrentCommand().getName());
    }

    public static Intake getInstance(){
        inst = inst == null ? new Intake() : inst;
        return inst;
    }

    public enum IntakePosition{
        Idle(Centimeters.of(0)), Out(Centimeters.of(10));
        Distance d;

        IntakePosition(Distance dist){
            d = dist;
        }

        public Distance getValue(){
            return d;
        }

        @Override
        public String toString(){
            return switch (this) {
                case Idle -> "Idle";
                case Out -> "Out";
            };
        }
    }
}
