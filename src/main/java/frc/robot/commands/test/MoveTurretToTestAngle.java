package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.util.DashboardGetter;

public class MoveTurretToTestAngle extends Command{
    private final Turret turret;
    private double turretTestAngle = 0;

    public MoveTurretToTestAngle(Turret turret){
        this.turret = turret;
        
        DashboardGetter.addGetDoubleData("Turret Test Angle", turretTestAngle, value -> turretTestAngle = value);
        
        addRequirements(turret);
    }

    @Override
    public void execute(){
        turret.setTargetAngle(Rotation2d.fromDegrees(turretTestAngle));
    }
}
