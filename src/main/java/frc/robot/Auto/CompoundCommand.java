package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;

public class CompoundCommand {
    private static Drivetrain drivetrain = Drivetrain.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
}
