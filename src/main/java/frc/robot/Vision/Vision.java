package frc.robot.Vision;

import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;
import frc.robot.Drivetrain.Drivetrain;

public class Vision {
    public List<PhotonCamera> cameras;
    public List<PhotonPoseEstimator> PoseEstimators;
    private static Vision inst;

    private Vision(){
        cameras = List.of(
            new PhotonCamera(Constants.FrontCamera.CameraName)
        );
        PoseEstimators = List.of(
            new PhotonPoseEstimator(Constants.Field, new Transform3d(Constants.FrontCamera.CameraPose.toMatrix()))
        );
    }

    public Optional<Pair<Pose2d, Double>> getPose(){
    Pair<Pose2d, Double> bestResult = null;
    if (!cameras.get(0).isConnected()) return Optional.empty();

    for (int i = 0; i < cameras.size(); i++) {
        PhotonCamera cam = cameras.get(i);
        var results = cam.getAllUnreadResults();

        if (results.isEmpty()) {
            continue;
        }

        var latest = results.get(results.size() - 1);
        var estimated = PoseEstimators
                .get(i)
                .estimateLowestAmbiguityPose(latest);

        if (estimated.isEmpty()) {
            continue;
        }

        var pose = estimated.get();
        
        // --- logging ---
        DogLog.log("Vision/" + cam.getName() + "/EstimatedPose",
                pose.estimatedPose);

        DogLog.log(
                "Vision/" + cam.getName() + "/Targets",
                pose.targetsUsed.stream()
                        .map(PhotonTrackedTarget::getFiducialId)
                        .map(id -> new Transform3d(
                                new Pose3d(Drivetrain.getInstance().getState().Pose),
                                Constants.Field.getTagPose(id).orElseThrow()
                        ))
                        .toArray(Transform3d[]::new)
        );

            bestResult = Pair.of(
                    pose.estimatedPose.toPose2d(),
                    Utils.getCurrentTimeSeconds()
            );
        }
    

    if (bestResult != null) {
        DogLog.log("Vision/BestPose", bestResult.getFirst());
        DogLog.log("Vision/BestTimestamp", bestResult.getSecond());
        return Optional.of(bestResult);
    }else return Optional.empty();


        
    }

    public static Vision getInstance(){
        inst = inst == null ? new Vision() : inst;
        return inst;
    }
}
