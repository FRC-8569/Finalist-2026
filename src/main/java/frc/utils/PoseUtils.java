package frc.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.stream.Stream;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivetrain.Drivetrain;

public class PoseUtils {
    public static Optional<FieldPlace> getRobotZone(){
        return Stream.of(FieldPlace.values()).filter(p -> p.contains(Drivetrain.getInstance().getState().Pose)).findFirst();
    }


    public static Pose2d getPose(Pose2d pose, Alliance alliance){ 
        var x = pose.getMeasureX();
        var delta = Constants.CenterLine.minus(x);
        SmartDashboard.putNumber("debug/deltaDistance", delta.in(Meters));

        boolean shouldMirror = alliance == Alliance.Blue ? delta.lt(Meters.of(0)) :  delta.gt(Meters.of(0)) ;
        SmartDashboard.putBoolean("debug/ShouldMirror", shouldMirror);

        if (!shouldMirror) return pose;

        var mirroredX = Constants.CenterLine.plus(delta);
        return new Pose2d(mirroredX, pose.getMeasureY(), Rotation2d.fromDegrees(-pose.getRotation().getDegrees()));
    }


    private class Constants {
        public static final Distance CenterLine = Meters.of(8.27052575);
        
    }

    public enum FieldPlace{
        BlueZone(4.652613,0),
        RedZone(16.541052,11.942413),
        Neutral(11.942413,4.652613);

        Distance Start, End;
        FieldPlace(double start, double end){
            Start = Meters.of(start);
            End = Meters.of(end);
        }

        public Pair<Distance, Distance> getValue(){
            return Pair.of(Start, End);
        }

        public boolean contains(Pose2d pose){
            return pose.getMeasureX().lte(Start) && pose.getMeasureX().gt(End);
        }
    }
}
