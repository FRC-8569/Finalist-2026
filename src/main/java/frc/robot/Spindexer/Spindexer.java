package frc.robot.Spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

        register();

        setDefaultCommand(feed().onlyIf(() -> (Shooter.getInstance().targetState.speedMetersPerSecond - Shooter.getInstance().getState().speedMetersPerSecond)/Shooter.getInstance().targetState.speedMetersPerSecond < 0.05 && Shooter.getInstance().targetState.speedMetersPerSecond > 5));
    }

    public Command feed(){
        return run(() -> {
            SpindexMotor.setControl(SpindexOut.withOutput(0.4));
            IndexMotor.setControl(SpindexOut.withOutput(0.4));
        }).handleInterrupt(() -> {
            SpindexMotor.stopMotor();
            IndexMotor.stopMotor();
        });
    }

    public static Spindexer getInstance(){
        inst = inst == null ? new Spindexer() : inst;
        return inst;
    }
}
