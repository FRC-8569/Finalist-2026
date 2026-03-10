package frc.robot.Spindexer;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Shooter.Shooter;
import frc.robot.Spindexer.Constants.Index;
import frc.robot.Spindexer.Constants.Spindex;

public class Spindexer implements Subsystem{
    public TalonFX SpindexMotor, IndexMotor;
    
    public DutyCycleOut SpindexOut, IndexOut;

    private TalonFXConfiguration SpindexConfig, IndexConfig;

    private static Spindexer inst;

    private Alert SpindexAlert, IndexAlert;

    private Spindexer(){
        SpindexMotor = new TalonFX(Spindex.MotorID, Spindex.bus);
        IndexMotor = new TalonFX(Index.MotorID, Index.bus);

        SpindexOut = new DutyCycleOut(0);
        IndexOut = new DutyCycleOut(0);

        SpindexConfig = new TalonFXConfiguration();
        IndexConfig = new TalonFXConfiguration();

        SpindexConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        IndexConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);

        IndexMotor.getConfigurator().apply(IndexConfig);
        SpindexMotor.getConfigurator().apply(SpindexConfig);

        IndexAlert = new Alert("Robot", "Index Motor Overheated", AlertType.kWarning);
        SpindexAlert = new Alert("Robot", "Spindex Motor Overheated", AlertType.kWarning);

        register();

        setDefaultCommand(feed());
   }

    public Command feed(){
        return runEnd(() -> {
                SpindexMotor.setControl(SpindexOut.withOutput(0.4));
                IndexMotor.setControl(IndexOut.withOutput(0.4));
            }
            , () -> {
            SpindexMotor.stopMotor();
            IndexMotor.stopMotor();
        }).onlyIf(() -> (MetersPerSecond.of(Shooter.getInstance().getState().speedMetersPerSecond).isNear(MetersPerSecond.of(Shooter.getInstance().targetState.speedMetersPerSecond), 0.1) && Shooter.getInstance().targetState.speedMetersPerSecond > Constants.VelocityDeadBand))
           .until(() -> !(MetersPerSecond.of(Shooter.getInstance().getState().speedMetersPerSecond).isNear(MetersPerSecond.of(Shooter.getInstance().targetState.speedMetersPerSecond), 0.1) && Shooter.getInstance().targetState.speedMetersPerSecond > Constants.VelocityDeadBand)) ;
    }

    @Override
    public void periodic(){
        DogLog.log("Debug/Spindexer/SpindexPercent", SpindexMotor.getDutyCycle().getValueAsDouble(), Percent);
        DogLog.log("Debug/Spindexer/IndexPercent", IndexMotor.getDutyCycle().getValueAsDouble(), Percent);
        DogLog.log("Driver/Spindexer/SpindexRunning", SpindexMotor.getDutyCycle().getValueAsDouble() > 0.1);
        DogLog.log("Driver/Spindexer/IndexRunning", IndexMotor.getDutyCycle().getValueAsDouble() > 0.1);
    
        IndexAlert.set(IndexMotor.getDeviceTemp().getValue().gt(Celsius.of(60)));
        SpindexAlert.set(SpindexMotor.getDeviceTemp().getValue().gt(Celsius.of(60)));
    }

    public List<Pair<Integer, Boolean>> isConnected(){
        return List.of(
            Pair.of(IndexMotor.getDeviceID(), IndexMotor.isConnected()),
            Pair.of(SpindexMotor.getDeviceID(), SpindexMotor.isConnected())
        );
    }

    public static Spindexer getInstance(){
        inst = inst == null ? new Spindexer() : inst;
        return inst;
    }
}
