package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.StackWalker.Option;
import java.util.Optional;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Shooter.Constants.Pitching;
import frc.robot.Shooter.Constants.Rolling;
import frc.robot.Shooter.Constants.index;

public class Shooter implements Subsystem{
    public TalonFX PitchMotor, ShootMotor, IndexingMotor;
    public CANcoder PitchEncoder;
    private static Shooter inst;

    public MotionMagicVoltage PitchPID;
    public MotionMagicVelocityVoltage ShootPID;
    public DutyCycleOut IndexPID;

    private TalonFXConfiguration PitchConfig, ShootConfig, IndexConfig;
    private CANcoderConfiguration PitchEncoderConfig;

    public Optional<SwerveModuleState> target;

    private Shooter(){
        PitchMotor = new TalonFX(Pitching.MotorID, Constants.bus);
        ShootMotor = new TalonFX(Rolling.MotorID, Constants.bus);
        IndexingMotor = new TalonFX(index.MotorID, Constants.bus);
        PitchEncoder = new CANcoder(Pitching.EncoderID, Constants.bus);

        PitchPID = new MotionMagicVoltage(0);
        ShootPID = new MotionMagicVelocityVoltage(0).withOverrideBrakeDurNeutral(true);
        IndexPID = new DutyCycleOut(0);
        target = Optional.empty();

        PitchConfig = new TalonFXConfiguration();
        ShootConfig = new TalonFXConfiguration();
        PitchEncoderConfig = new CANcoderConfiguration();
        IndexConfig = new TalonFXConfiguration();

        // TODO: Software Limit Switch TBD
        PitchConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        PitchConfig.Feedback
            .withRotorToSensorRatio(Pitching.PitchingGearRatio)
            .withFusedCANcoder(PitchEncoder);
        PitchConfig.withSlot0(Pitching.PitchPID);
        PitchConfig.withMotionMagic(Pitching.PitchingMagic);

        IndexConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        
        ShootConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        ShootConfig.Feedback
            .withSensorToMechanismRatio(Pitching.PitchingGearRatio);
        ShootConfig.withSlot0(Rolling.RollingPID);
        ShootConfig.withMotionMagic(Rolling.RollingMagic);

        PitchEncoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        PitchMotor.getConfigurator().apply(PitchConfig);
        ShootMotor.getConfigurator().apply(ShootConfig);
        PitchEncoder.getConfigurator().apply(PitchEncoderConfig);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            MetersPerSecond.of(ShootMotor.getVelocity().getValue().in(RotationsPerSecond)*Rolling.WheelCirc.in(Meters)),
            Rotation2d.fromRotations(ShootMotor.getPosition().getValueAsDouble())
        );
    }

    public Command shoot(SwerveModuleState state){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                run(() -> PitchMotor.setControl(PitchPID.withPosition(state.angle.getRotations()))).until(() -> PitchMotor.getPosition().getValue().minus(PitchPID.getPositionMeasure()).abs(Degree) < 0.5),
                run(() -> ShootMotor.setControl(ShootPID.withVelocity(state.speedMetersPerSecond/Rolling.WheelCirc.in(Meters)))).until(() -> PitchMotor.getVelocity().getValue().minus(ShootPID.getVelocityMeasure()).abs(RotationsPerSecond) < 0.5/Rolling.WheelCirc.in(Meters))
            ),
            run(() -> IndexingMotor.setControl(IndexPID.withOutput(0.6))).withTimeout(Seconds.of(2)) //TODO: YOLO fuel detection implement
        ).finallyDo(() -> {
            PitchMotor.setControl(PitchPID.withPosition(0));
            ShootMotor.stopMotor();
            IndexingMotor.stopMotor();
        });
    }

    public Command resetPose(){
        return runOnce(() -> PitchEncoder.setPosition(0));
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
}
