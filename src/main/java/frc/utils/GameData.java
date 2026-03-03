package frc.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.ShootUtils.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Drivetrain;

public class GameData implements Subsystem{
    public Optional<FieldObjects> CurrentLocking = Optional.empty();
    private static GameData inst;
    private boolean manualCancel = false;

    public void withLocking(FieldObjects obj){
        this.CurrentLocking = Optional.of(obj);
    }

    public void clearLocking(){
        resetLocking();
        manualCancel = true;
    }

    private void resetLocking(){
        this.CurrentLocking = Optional.empty();
    }

    public boolean isHubActive(){
        return switch (DriverStation.getGameSpecificMessage()) {
            case "R" -> DriverStation.getAlliance().orElseThrow() == Alliance.Red;
            case "B" -> DriverStation.getAlliance().orElseThrow() == Alliance.Blue;
            default -> false;
        };
    }

    public Optional<RobotState> predictRobotState(FieldObjects obj){
        return predictRobotState(obj.getPose());
    }

    public Optional<RobotState> predictRobotState(Pose3d pose){
        return ShootUtils.calculateState(new Transform3d(new Pose3d(Drivetrain.getInstance().getState().Pose), pose));
    }

    @Override
    public void periodic(){
        if(isHubActive() && predictRobotState().isPresent() && !manualCancel && Drivetrain.getInstance().inZone()) CurrentLocking = Optional.of(FieldObjects.HUB);
        
        if(!Drivetrain.getInstance().inZone() && manualCancel) manualCancel = false;
    }

    /**
     * This function will return the predicted robot state of shooting to the HUB, for general use, please see {@link #predictRobotState(FieldObjects)} or {@link #predictRobotState(Pose3d)}
     * @return
     */
    public Optional<RobotState> predictRobotState(){
        if(!Drivetrain.getInstance().inZone()) return Optional.empty();
        return predictRobotState(FieldObjects.HUB);
    }

    public static GameData getInstance(){
        inst = inst == null ? new GameData() : inst;
        return inst;
    }
}