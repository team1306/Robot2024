package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.MotorUtil;

import java.util.function.DoubleSupplier;

public class ToggleShooterCommand extends Command{
    private final Shooter shooter;
    private boolean finished = false;
    private final DoubleSupplier speedSupplier;

    public ToggleShooterCommand(DoubleSupplier speedSupplier, Shooter shooter){
        addRequirements(shooter);
        this.shooter = shooter;
        this.speedSupplier = speedSupplier;
    }
    @Override
    public void initialize(){
        System.out.println("shooter on");
        finished = false;
        shooter.setTargetSpeed(MotorUtil.clampPercent(speedSupplier.getAsDouble()));
    }
    @Override
    public void end(boolean interrupted){
        shooter.setTargetSpeed(0);
        System.out.println("Shooter no spinny spinny");
    }

    public void stop() {
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
