package frc.robot.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.GlobalConstants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Vision.Constants.RearCamera;

public class Vision implements Subsystem{
    private static Vision inst;
    private static PhotonCamera BackCamera = new PhotonCamera(RearCamera.CameraName);
    public PhotonPoseEstimator BackPoseEstimator;
    public Optional<Pair<Pose2d, Double>> result = Optional.empty();

    private record EstimatedResult(Pose3d rawPose, double timestamp, List<Transform3d> targets ) implements Sendable{
        @Override
        public void initSendable(SendableBuilder builder){
            
        }
    }

    private Optional<PhotonPipelineResult> currentResult = Optional.empty();

    private Vision(){
        
        BackPoseEstimator = new PhotonPoseEstimator(GlobalConstants.Field, RearCamera.CameraPlace);
        register();
    }

    public Optional<Pair<Pose2d, Double>> getRobotPose(){
        Optional<EstimatedResult> res = getPose();
        if(res.isEmpty()) {
            DogLog.log("Driver/Vision/TargetUsed", "null");
            return Optional.empty();
        }
        EstimatedResult r = res.get();
        DogLog.log("Driver/Vision/TargetUsed", r.targets.stream().<Pose3d>map(t -> new Pose3d(Drivetrain.getInstance().getState().Pose).plus(t)).toArray(Pose3d[]::new));
        return Optional.of(Pair.of(r.rawPose.toPose2d(), Utils.getCurrentTimeSeconds()));
    }

    /**
     * Get the estimated pose by PhotonVision
     * @return
     */
    public Optional<EstimatedResult> getPose(){
        return currentResult
        .flatMap(BackPoseEstimator::estimateAverageBestTargetsPose)
        .map(e -> new EstimatedResult(e.estimatedPose, e.timestampSeconds, e.targetsUsed.stream().map(PhotonTrackedTarget::getBestCameraToTarget).toList()));
    }

    @Override
    public void periodic(){
        List<PhotonPipelineResult> res = BackCamera.getAllUnreadResults();
        currentResult = !res.isEmpty() ? Optional.of(res.get(res.size() - 1)) : Optional.empty();
        result = getRobotPose();

        log();
    }

    private void log(){
        getPose().ifPresentOrElse(r -> {
            DogLog.log("Debug/Vision/EstimatedPose", r.rawPose.toPose2d());
            DogLog.log("Debug/Vision/TimestampSecond", r.timestamp);
        }, () -> {
            DogLog.log("Debug/Vision/EstimatedPose", "null");
            DogLog.log("Debug/Vision/TimestampSecond", "null");
        });
    }

    public static Vision getInstance(){
        inst = (inst == null && BackCamera.isConnected()) ? new Vision() : inst;
        return inst;
    }
}
