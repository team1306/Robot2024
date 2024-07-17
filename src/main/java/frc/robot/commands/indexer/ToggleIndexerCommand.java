package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class ToggleIndexerCommand extends Command{

    private final Indexer indexer;
    private final DoubleSupplier speed;
    
    public ToggleIndexerCommand (Indexer indexer, DoubleSupplier speed) {
        this.indexer = indexer;
        this.speed = speed;
    } 

    @Override
    public void execute () {
       indexer.setTargetSpeed(speed.getAsDouble());
    }

    @Override
    public void end (boolean interrupted) {
        indexer.setTargetSpeed(0);
    }
}