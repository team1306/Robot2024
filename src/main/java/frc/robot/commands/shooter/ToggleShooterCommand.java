package frc.robot.commands.shooter;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.MotorUtil;

public class ToggleShooterCommand extends Command{
    private final Shooter shooter;
    private boolean finished = false;
    private final DoubleSupplier speedSupplier;
    private final DoubleSupplier angleSupplier;
    private final HashMap<Integer, Double> shooterSpeedLookupFromAngle = new HashMap<>() {{
        put(0, .79);
        put(1, .79);
        put(2, .79);
        put(3, .89);
        put(4, .89);
        put(5, 1.0);
        put(6, 1.0);
        put(7, 1.0);
        put(8, 1.0);
    }};

    public ToggleShooterCommand(DoubleSupplier speedSupplier, DoubleSupplier angleSupplier, Shooter shooter){
        addRequirements(shooter);
        this.shooter = shooter;
        this.speedSupplier = speedSupplier;
        this.angleSupplier = angleSupplier;
    }
    @Override
    public void initialize(){
        System.out.println("shooter on");
        shooter.setTargetSpeed(MotorUtil.clampPercent(speedSupplier.getAsDouble() * /*shooterSpeedLookupFromAngle.get(((int)angleSupplier.getAsDouble()) / 10)*/ 1)); 
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
