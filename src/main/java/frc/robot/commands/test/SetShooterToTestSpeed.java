package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.DashboardGetter;

public class SetShooterToTestSpeed extends Command{
    private final Shooter shooter;
    private double speed = 0;
    
    public SetShooterToTestSpeed(Shooter shooter) {
        this.shooter = shooter;

        DashboardGetter.addGetDoubleData("Indexer Test Speed", speed, value -> speed = value);    
    }

    @Override
    public void execute() {
        shooter.setTargetSpeed(speed);
    }
}
