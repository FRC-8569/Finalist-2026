package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Constants.Index;
import frc.robot.Shooter.Constants.Pitch;
import frc.robot.Shooter.Constants.Shoot;
import frc.robot.Shooter.Constants.Spindex;
import frc.utils.FieldObjects;
import frc.utils.PoseUtils;
import frc.utils.ShootUtils;
import frc.utils.PoseUtils.FieldPlace;
import frc.utils.ShootUtils.RobotState;

public class Shooter implements Subsystem{
    public static Shooter inst;
    public TalonFX ShootingMotor, PitchingMotor, IndexingMotor, SpindexingMotor;
    public CANcoder PitchingEncoder;

    public MotionMagicTorqueCurrentFOC PitchingPID;
    public MotionMagicVelocityTorqueCurrentFOC ShootPID, IndexPID, SpindexPID;

    private TalonFXConfiguration ShootConfig, PitchConfig, IndexConfig, SpindexConfig;
    private CANcoderConfiguration PitchEncoderConfig;
    private StringSubscriber CurrentLocking;

    public boolean hasControlled = false;
    public boolean shootable = false;

    private Shooter(){
        ShootingMotor = new TalonFX(Shoot.MotorID, Constants.bus);
        PitchingMotor = new TalonFX(Pitch.MotorID, Constants.bus);
        IndexingMotor = new TalonFX(Index.MotorID, Constants.bus);
        SpindexingMotor = new TalonFX(Spindex.MotorID, Constants.bus);
        PitchingEncoder = new CANcoder(Pitch.EncoderID, Constants.bus);

        PitchingPID = new MotionMagicTorqueCurrentFOC(0);
        ShootPID = new MotionMagicVelocityTorqueCurrentFOC(0);
        IndexPID = new MotionMagicVelocityTorqueCurrentFOC(0);
        SpindexPID = new MotionMagicVelocityTorqueCurrentFOC(0);

        ShootConfig = new TalonFXConfiguration();
        PitchConfig = new TalonFXConfiguration();
        IndexConfig = new TalonFXConfiguration();
        SpindexConfig = new TalonFXConfiguration();
        PitchEncoderConfig = new CANcoderConfiguration();

        PitchConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        PitchConfig.Feedback
            .withFusedCANcoder(PitchingEncoder)
            .withRotorToSensorRatio(Pitch.GearRatio);
        PitchConfig.withSlot0(Pitch.PitchPID);
        PitchConfig.withMotionMagic(Pitch.PitchMagic);
        PitchConfig.withSoftwareLimitSwitch(Pitch.PitchLimit);

        ShootConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        ShootConfig.Feedback
            .withSensorToMechanismRatio(Shoot.GearRatio);
        ShootConfig.withSlot0(Shoot.ShootPID);
        ShootConfig.withMotionMagic(Shoot.ShootMagic);

        IndexConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        IndexConfig.Feedback
            .withSensorToMechanismRatio(Index.GearRatio);
        IndexConfig.withSlot0(Index.IndexPID);
        IndexConfig.withMotionMagic(Index.IndexMagic);

        SpindexConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        SpindexConfig.Feedback
            .withSensorToMechanismRatio(Spindex.GearRatio);
        SpindexConfig.withSlot0(Spindex.SpindexPID);
        SpindexConfig.withMotionMagic(Spindex.SpindexMagic);

        PitchEncoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(Pitch.PitchOffset);

        PitchingMotor.getConfigurator().apply(PitchConfig);
        ShootingMotor.getConfigurator().apply(ShootConfig);
        IndexingMotor.getConfigurator().apply(IndexConfig);
        SpindexingMotor.getConfigurator().apply(SpindexConfig);
        PitchingEncoder.getConfigurator().apply(PitchEncoderConfig);

        CurrentLocking = NetworkTableInstance.getDefault().getStringTopic("Drivetrain/LockingTarget").subscribe("null");

        register();


    }

    public SwerveModuleState getShooterState(){
        return new SwerveModuleState(
            MetersPerSecond.of(ShootingMotor.getVelocity().getValue().in(RotationsPerSecond)*Shoot.WheelCirc.in(Meters)), 
            new Rotation2d(ShootingMotor.getPosition().getValue()));
    }

    public Command setState(SwerveModuleState state){
        return run(() -> {
            PitchingMotor.setControl(PitchingPID.withPosition(state.angle.getMeasure()));
            ShootingMotor.setControl(ShootPID.withVelocity(state.speedMetersPerSecond/Shoot.WheelCirc.in(Meters)));
        }).until(() -> PitchingMotor.getPosition().getValue().isNear(PitchingPID.getPositionMeasure(), 0.05) && ShootingMotor.getVelocity().getValue().isNear(ShootPID.getVelocityMeasure(), 0.05));
    }

    public Command indexFuel(){
        return Commands.run(() -> {
            SpindexingMotor.setControl(SpindexPID.withVelocity(1));
            IndexingMotor.setControl(IndexPID.withVelocity(1));
        }).handleInterrupt(() -> {
            SpindexingMotor.stopMotor();
            IndexingMotor.stopMotor();
        });
    }

    public Command shoot(Angle angle, LinearVelocity vel){
        return setState(new SwerveModuleState(vel,new Rotation2d(angle))).alongWith(Commands.runOnce(() -> hasControlled = true)).andThen(indexFuel()).handleInterrupt(() -> emergencyStop().schedule());
    }

    public Command emergencyStop(){
        return runOnce(() -> {
            SpindexingMotor.stopMotor();
            IndexingMotor.stopMotor();
            setState(new SwerveModuleState(0, Rotation2d.kZero));
        });
    }

    @Override
    public void periodic(){
            Optional<FieldObjects> currentLocking = CurrentLocking.get().contains("null") ? Optional.empty() : Optional.of(FieldObjects.valueOf(CurrentLocking.get()));
            Optional<RobotState> state = Optional.empty();
            if(currentLocking.isPresent()){
                state = ShootUtils.calculateState(new Transform3d(new Pose3d(Drivetrain.getInstance().getState().Pose).plus(Constants.ShooterPlace), currentLocking.get().getPose()));
            }

            state.ifPresent((s) -> {
                if(!hasControlled && PoseUtils.getRobotZone().get() != FieldPlace.Neutral) setState(new SwerveModuleState(s.vel(), new Rotation2d(s.pitch()))).schedule();
            });
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
}
