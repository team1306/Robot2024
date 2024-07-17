package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.util.DashboardGetter;

public class SetIndexerToTestSpeed extends Command{
    private final Indexer indexer;
    private double speed = 0;
    
    public SetIndexerToTestSpeed(Indexer indexer) {
        this.indexer = indexer;

        DashboardGetter.addGetDoubleData("Indexer Test Speed", speed, value -> speed = value);    
    }

    @Override
    public void execute() {
        indexer.setTargetSpeed(speed);
    }
}