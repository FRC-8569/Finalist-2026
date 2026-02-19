package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.GlobalConstant;
import frc.robot.Drivetrain.Drivetrain;

public enum FieldObjects {
    HUB(new Pose2d()), 
    Alliance(new Pose2d());

    FieldObjects(Pose2d pose){

    }

    public Pose3d getPose(){
        return switch (this) {
            case HUB -> new Pose3d(PoseUtils.getPose(GlobalConstant.HubPose.toPose2d(), DriverStation.getAlliance().orElseThrow()).rotateBy(Rotation2d.k180deg).getX(), PoseUtils.getPose(GlobalConstant.HubPose.toPose2d(), DriverStation.getAlliance().orElseThrow()).rotateBy(Rotation2d.k180deg).getY(), GlobalConstant.HubPose.getZ(), new Rotation3d(PoseUtils.getPose(GlobalConstant.HubPose.toPose2d(), DriverStation.getAlliance().orElseThrow()).rotateBy(Rotation2d.k180deg).getRotation()));
            case Alliance -> new Pose3d(PoseUtils.getPose(new Pose2d(14.5, Drivetrain.getInstance().getState().Pose.getY(), Rotation2d.kZero), DriverStation.getAlliance().orElseThrow()));
        };
    }
}
