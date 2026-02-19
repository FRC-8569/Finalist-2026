package frc.robot.PowerManageMent;

import java.util.List;
import java.util.stream.IntStream;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerManageMent extends SubsystemBase{
    public PowerDistribution pdh;

    public List<Pair<String,Integer>> Drivetrain, Intake, Shooter;

    private static PowerManageMent inst;

    private PowerManageMent(){
        pdh = new PowerDistribution(15, ModuleType.kRev);
        DogLog.setPdh(pdh);
        register();

        Drivetrain = IntStream.range(0, Constants.Drivetrain.length).mapToObj(i -> Pair.of(Constants.DrivetrainName[i], Constants.Drivetrain[i])).toList();
        Intake = IntStream.range(0, Constants.Intake.length).mapToObj(i -> Pair.of(Constants.IntakeName[i], Constants.Intake[i])).toList();
        Shooter = IntStream.range(0, Constants.Shooter.length).mapToObj(i -> Pair.of(Constants.ShooterName[i], Constants.Shooter[i])).toList();
    }

    @Override
    public void periodic(){
        Drivetrain.stream().forEach((t) -> DogLog.log("PowerManageMents/Drivetrain/%s".formatted(t.getFirst()), pdh.getCurrent(t.getSecond())*pdh.getVoltage()));
        Intake.stream().forEach((t) -> DogLog.log("PowerManageMents/Intake/%s".formatted(t.getFirst()), pdh.getCurrent(t.getSecond())));
        Shooter.stream().forEach((t) -> DogLog.log("PowerManageMents/Shooter/%s".formatted(t.getFirst()), pdh.getCurrent(t.getSecond())));
    }

    public static PowerManageMent getInstance(){
        inst = inst == null ? new PowerManageMent() : inst;
        return inst;
    }
}
