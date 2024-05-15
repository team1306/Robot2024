package frc.robot.commands.outreach;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.DashboardGetter;

public class ShooterSpeedCommand extends Command{
    private final Shooter shooter;
    private final DoubleSupplier speedSupplier;
    private static double maxSpeed = 0.7;

    public ShooterSpeedCommand(Shooter shooter, DoubleSupplier speedSupplier){
        this.shooter = shooter;
        this.speedSupplier = speedSupplier;
        DashboardGetter.addGetDoubleData("Max shooter speed", maxSpeed, value -> maxSpeed = value);
        addRequirements(shooter);
    }

    public void execute(){
        shooter.setTargetSpeed(MathUtil.clamp(speedSupplier.getAsDouble(), 0, maxSpeed));
    }

    public void end(boolean interrupted){
        shooter.setTargetSpeed(0);
    }
    
}
