package frc.utils;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.List;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Drivetrain.Drivetrain;

public enum FieldObjects {
    HUB,CurrentAlliance, Tower;

    public Pose3d getPose(){
        Alliance a = DriverStation.getAlliance().orElseThrow();
        DogLog.log("Debug/Drivetrain/DeterminingPose", Drivetrain.getInstance().getState().Pose);
        return switch(this){
            case HUB -> new Pose3d(Centimeters.of(465),Centimeters.of(403),Centimeters.of(175), new Rotation3d(Degrees.of(0),Degrees.of(-90),Degrees.of(0)));
            case CurrentAlliance -> new Pose3d(PoseUtils.getPose(Drivetrain.getInstance().getState().Pose, a, null));
            case Tower -> new Pose3d(Drivetrain.getInstance().getState().Pose.nearest(List.of(PoseUtils.getPose(new Pose2d(15.5,3.4,Rotation2d.kZero), a, null),PoseUtils.getPose(new Pose2d(15.5,5.2,Rotation2d.k180deg), a, null))));
        };
    }

    public Supplier<Pose3d> getSupplier(){
        Alliance a = DriverStation.getAlliance().orElseThrow();
        DogLog.log("Debug/Drivetrain/DeterminingPose", Drivetrain.getInstance().getState().Pose);
        return () -> switch(this){
            case HUB -> new Pose3d(Centimeters.of(465),Centimeters.of(403),Centimeters.of(175), new Rotation3d(Degrees.of(0),Degrees.of(-90),Degrees.of(0)));
            case CurrentAlliance -> new Pose3d(PoseUtils.getPose(Drivetrain.getInstance().getState().Pose, a, null));
            case Tower -> new Pose3d(Drivetrain.getInstance().getState().Pose.nearest(List.of(PoseUtils.getPose(new Pose2d(1.55, 3.32,Rotation2d.k180deg), a, null),PoseUtils.getPose(new Pose2d(1.55,4.17,Rotation2d.k180deg), a, null))));
        };
    }

    public Supplier<Pose2d> get2dSupplier(){
        return () -> getSupplier().get().toPose2d();
    }
}
