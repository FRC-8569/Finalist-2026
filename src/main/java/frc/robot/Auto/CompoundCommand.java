package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.utils.PoseUtils.ShootPreset;
import frc.utils.PoseUtils.Side;

public class CompoundCommand {
    private static Drivetrain drivetrain = Drivetrain.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();

    public static Command shootFuel(ShootPreset set){
        return drivetrain.drive(set.getTargetPose(), null).alongWith(shooter.setShooterState(set.getTargetVelocity()))
                .andThen(shooter.setState(set.getState()));
    }

    public static Command intakeFuel(Pose2d pose, Side side){
        return drivetrain.drive(pose, side)
                    .raceWith(intake.intake(true).beforeStarting(intake.moveIntake(true)));
    }
}
