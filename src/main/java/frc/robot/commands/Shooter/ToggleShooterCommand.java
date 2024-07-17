package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ToggleShooterCommand extends Command{

    private final Shooter shooter;
    private final DoubleSupplier speed;
    
    public ToggleShooterCommand (Shooter shooter, DoubleSupplier speed) {
        this.shooter = shooter;
        this.speed = speed;
    }
 
    @Override
    public void execute () {
       shooter.setTargetSpeed(speed.getAsDouble());
    }

    @Override
    public void end (boolean interrupted) {
        shooter.setTargetSpeed(0);
    }
}