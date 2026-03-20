package frc.robot.Auto;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.utils.Tools;
import frc.utils.Tools.FieldSide;

public class Auto {
    public static Drivetrain drivetrain = Drivetrain.getInstance();
    public static Intake intake = Intake.getInstance();
    public static Shooter shooter = Shooter.getInstance();
    public static Timer timer = new Timer();

    public static Command getAutoCommand(){
        DogLog.log("Autonomous/DeterminingPose", drivetrain.getState().Pose);
        return drivetrain.getSide() == FieldSide.RIGHT ? 
            new SequentialCommandGroup(
                CompoundCommand.shoot(Tools.RightBump, Seconds.of(3)),
                drivetrain.drive(new Pose2d(16.0,7.2,Rotation2d.kCCW_90deg), null),
                new WaitCommand(Seconds.of(5)),
                CompoundCommand.shoot(Tools.RightBump)
            ) : 
            new SequentialCommandGroup(
                CompoundCommand.shoot(Tools.LeftBump, Seconds.of(3))
            );
    }
}
