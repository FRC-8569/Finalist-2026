package frc.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.stream.Stream;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.GlobalConstants;
import frc.robot.Drivetrain.Drivetrain;
import frc.utils.ShootUtils.RobotState;

public class PoseUtils {
    public static Optional<FieldPlace> getRobotZone(){
        return Stream.of(FieldPlace.values()).filter(p -> p.contains(Drivetrain.getInstance().getState().Pose)).findFirst();
    }


    public static Pose2d getPose(Pose2d pose, Alliance alliance, Side side){
        Pair<Distance, Distance> delta = Pair.of(GlobalConstants.CenterLine.getFirst().minus(pose.getMeasureX()), GlobalConstants.CenterLine.getSecond().minus(pose.getMeasureY()));
        Pair<Boolean, Boolean> shouldMirror = Pair.of(alliance == Alliance.Blue ? delta.getFirst().lt(Meters.zero()) : delta.getFirst().gt(Meters.zero()), 
                                                    side != null ? ((side == Side.LEFT) == ((alliance == Alliance.Blue) == delta.getSecond().lt(Meters.zero()))) : false);

        return new Pose2d(
            shouldMirror.getFirst() ? GlobalConstants.CenterLine.getFirst().plus(delta.getFirst()) : pose.getMeasureX(), 
            shouldMirror.getSecond() ? GlobalConstants.CenterLine.getSecond().plus(delta.getSecond()) : pose.getMeasureY(),
            shouldMirror.getFirst() ? pose.getRotation().unaryMinus() : pose.getRotation());
    }


    public enum Side{
        LEFT, RIGHT;
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

    public static ShootPreset InFrontOfHub = new ShootPreset(new Pose2d(13.25,4.1, Rotation2d.k180deg), new RobotState(Rotation2d.k180deg, MetersPerSecond.of(15), Degrees.of(10)));

    public record ShootPreset(Pose2d robotState, RobotState state) {
        public Pose2d getTargetPose(){
            return new Pose2d(robotState.getX(), robotState.getY(), state.facing());
        }

        public LinearVelocity getTargetVelocity(){
            return state.vel();
        }

        public Angle getPitchAngle(){
            return state.pitch();
        }

        public SwerveModuleState getState(){
            return new SwerveModuleState(state.vel(), new Rotation2d(state.pitch()));
        }
    }
}
