package frc.robot.commands.outreach;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterSpeedCommand extends Command{

    private final Shooter shooter;
    private final DoubleSupplier speedSupplier;

    public ShooterSpeedCommand(Shooter shooter, DoubleSupplier speedSupplier){
        this.shooter = shooter;
        this.speedSupplier = speedSupplier;
        addRequirements(shooter);
    }

    public void execute(){
        shooter.setTargetSpeed(speedSupplier.getAsDouble());
    }

    public void end(boolean interrupted){
        shooter.setTargetSpeed(0);
    }
    
}
