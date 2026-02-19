package frc.robot.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.Optional;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import frc.robot.Intake.Constants.Tongue;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Constants.Roller;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    public TalonFX TongueMotor, RollerMotor;
    private static Intake inst;

    private TalonFXConfiguration TongueConfig, RollerConfig;
   
    public MotionMagicVoltage LickingPID;
    public MotionMagicVelocityVoltage RollingPID;

    private boolean hasStalled = false;
    private Optional<IntakePosition> target = Optional.empty();

    private Intake(){
        TongueMotor = new TalonFX(Tongue.MotorID, Constants.bus);
        RollerMotor = new TalonFX(Roller.MotorID, Constants.bus);
        
        TongueConfig = new TalonFXConfiguration();
        RollerConfig = new TalonFXConfiguration();
        
        LickingPID = new MotionMagicVoltage(0);
        RollingPID = new MotionMagicVelocityVoltage(0);

        TongueConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        TongueConfig.Feedback
            .withSensorToMechanismRatio(Tongue.GearRatio);
        TongueConfig.withSlot0(Tongue.TonguePID);
        TongueConfig.withMotionMagic(Tongue.TongueMagic);
        TongueConfig.SoftwareLimitSwitch
            .withForwardSoftLimitThreshold(1.45)
            .withReverseSoftLimitThreshold(0)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true);

        RollerConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        RollerConfig.Feedback
            .withSensorToMechanismRatio(Roller.GearRatio);
        RollerConfig.withSlot0(Roller.RollerPID);
        RollerConfig.withMotionMagic(Roller.RollerMagic);

        TongueMotor.getConfigurator().apply(TongueConfig);
        RollerMotor.getConfigurator().apply(RollerConfig);
        this.register();  
    }

    public Angle getIntakePosition(){
        return TongueMotor.getPosition().getValue();
    }

    public LinearVelocity getRollerVelocity(){
        return MetersPerSecond.of(RollerMotor.getVelocity().getValue().in(RotationsPerSecond)*Roller.WheelCirc.in(Meters));
    }

    public Command moveIntake(IntakePosition position){
        return run(() -> {
            TongueMotor.setControl(LickingPID.withPosition(position.getValue()));
            target = Optional.of(position);
        }).until(() -> TongueMotor.getVelocity().getValue().abs(RotationsPerSecond) < 0.01 && getIntakePosition().isNear(position.getValue(), 0.05))
            .finallyDo(() -> {
                TongueMotor.setControl(position == IntakePosition.Out ? new CoastOut(): new StaticBrake());
                target = Optional.empty();
            }).withName("MovingIntake").beforeStarting(CalibrateIntake().onlyIf(() -> hasStalled));
    }

    public Command runIntake(LinearVelocity vel){
        return run(() -> {
            LinearVelocity v = vel.lt(Roller.SpeedRange[0]) ? Roller.SpeedRange[0] : vel;
            v = v.gt(Roller.SpeedRange[1]) ? Roller.SpeedRange[1] : v;
            RollerMotor.setControl(RollingPID.withVelocity(v.in(MetersPerSecond)/Roller.WheelCirc.in(Meters)));
        });
    }

    public Command intake(boolean isIntake){
        ChassisSpeeds speeds = Drivetrain.getInstance().getState().Speeds;
        return runIntake(MetersPerSecond.of(Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2)+Math.pow(speeds.vxMetersPerSecond, 2))).times(2.2).times(isIntake ? 1 : -1)).handleInterrupt(() -> RollerMotor.stopMotor());
    }

    public Command resetPose(){
        return runOnce(() -> TongueMotor.setPosition(0)).finallyDo(() -> idle());
    } 

    public Command CalibrateIntake(){
        return runEnd(() -> { 
            TongueMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withReverseSoftLimitEnable(false));
            TongueMotor.set(-0.25);
            }, () -> {
            TongueMotor.stopMotor();
            TongueMotor.setPosition(0);
            TongueMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withReverseSoftLimitEnable(true));
            hasStalled = false;
        }).until(() -> TongueMotor.getStatorCurrent().getValue().gt(Amps.of(9)));
    }

    public static Intake getInstance(){
        inst = inst == null ? new Intake() : inst;
        return inst;
    }

    private boolean isMotorStalling(){
        return TongueMotor.getStatorCurrent().getValue().abs(Amps) > Tongue.StallLimit.in(Amps);
    }

    @Override
    public void periodic(){
        DogLog.log("Intake/IntakePositionPrecise", getIntakePosition());
        if(getCurrentCommand() != null)DogLog.log("Intake/CurrentDoing", getCurrentCommand().getName());
        DogLog.log("Intake/TongueControlMode", TongueMotor.getControlMode().getValue().toString());
        DogLog.log("Intake/RollerControlMode", RollerMotor.getControlMode().getValue().toString());
        DogLog.log("Intake/RollerVelocity", getRollerVelocity());

        DogLog.log("Intake/TongueCurrents", TongueMotor.getStatorCurrent().getValue());
        DogLog.log("Intake/TongueVelocity", TongueMotor.getVelocity().getValue());

        DogLog.log("Intake/Target", target.isPresent() ? target.get().toString() : "null");
        if(DriverStation.isDisabled() && TongueMotor.getControlMode().getValue() != ControlModeValue.CoastOut) TongueMotor.setControl(new CoastOut());
        if(isMotorStalling()) hasStalled = true;
    }

    public enum IntakePosition{
        Idle(Rotations.of(0)), Out(Rotations.of(1.4));
        Angle d;

        IntakePosition(Angle dist){
            d = dist;
        }

        public Angle getValue(){
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
