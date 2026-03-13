package frc.robot.Auto;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.utils.Tools;
import frc.utils.Tools.FieldSide;
import frc.utils.Tools.RobotState;

public class CompoundCommand {
    private static Drivetrain drivetrain = Drivetrain.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();

    public static Command shoot(Pose2d pose, FieldSide side){
        pose = Tools.getPose(pose, side);
        RobotState state = RobotState.create(new Transform3d(new Pose3d(pose), Tools.HUB.getPose()));
        return new SequentialCommandGroup(
            shooter.setShooterState(MetersPerSecond.of(state.getShootingState().speedMetersPerSecond)).deadlineFor(drivetrain.drive(new Pose2d(pose.getX(), pose.getY(),state.drivetrainFacing()), null)),
            shooter.setState(state)
        );
    }
}
