package frc.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameData {
    Optional<Alliance> ActiveAllaince;
    int FuelScore, TowerScore;

    public GameData(Optional<Alliance> ActiveAlliance, int FuelScore, int TowerScore){
        this.ActiveAllaince = ActiveAlliance;
        this.FuelScore = FuelScore;
        this.TowerScore = TowerScore;
    }

    // public GameData.ExtendedAlliance getActiveAlliance(){
    //     DriverStation.getGameSpecificMessage();
    // }

    public enum ExtendedAlliance{
        Red, Blue, Both;
        public Optional<ExtendedAlliance> fromAlliance(Optional<Alliance> alliance){
            return alliance.isPresent() ? Optional.of((alliance.get() == Alliance.Red ? Red : Blue)) : Optional.empty();
        }

        public Optional<Alliance> getAlliance(){
            return switch(this){
                case Red -> Optional.of(Alliance.Red);
                case Blue -> Optional.of(Alliance.Blue);
                case Both -> Optional.empty();
            };
        }
    }
}
