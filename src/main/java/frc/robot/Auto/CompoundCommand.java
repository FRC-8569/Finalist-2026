package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.utils.GameData;
import frc.utils.PoseUtils.ShootPreset;
import frc.utils.PoseUtils.Side;
import frc.utils.ShootUtils.RobotState;

public class CompoundCommand {
    private static Drivetrain drivetrain = Drivetrain.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();

    public Command shootFuel(ShootPreset set){
        return drivetrain.drive(set.getTargetPose(), null)
                .andThen(shooter.setState(set.getState()));
    }

    public static Command intakeFuel(Pose2d pose, Side side){
        return drivetrain.drive(pose, side)
                    .alongWith(shooter.setState(new SwerveModuleState(3, Rotation2d.fromDegrees(20))))
                    .raceWith(intake.intake(true).beforeStarting(intake.moveIntake(true)));
    }
}
