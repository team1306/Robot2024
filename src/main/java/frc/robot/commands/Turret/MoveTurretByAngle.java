package frc.robot.commands.Turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class MoveTurretByAngle extends Command{

    private final Turret turret;
    private final Rotation2d angleDifference;

    public MoveTurretByAngle(Turret turret, Rotation2d angleDifference){
        this.turret = turret;
        this.angleDifference = angleDifference;
    }

    @Override
    public void initialize(){
        double scoreLeft = scoreFunction(
            turret.getCurrentAngle().getDegrees() + angleDifference.getDegrees() - 360, 
            Math.abs(angleDifference.getDegrees() - 360));

        double scoreCenter = scoreFunction(
            turret.getCurrentAngle().getDegrees() + angleDifference.getDegrees(), 
            Math.abs(angleDifference.getDegrees()));

        double scoreRight = scoreFunction(
            turret.getCurrentAngle().getDegrees() + angleDifference.getDegrees() + 360, 
            Math.abs(angleDifference.getDegrees() + 360));

        if(angleDifference.getDegrees() % 360 == 0) return;

        if(scoreLeft > scoreCenter && scoreLeft > scoreRight)
            setTurretLeftAngle();
        else if(scoreCenter > scoreLeft && scoreCenter > scoreRight)
            setTurretCenterAngle();
        else if(scoreRight > scoreCenter && scoreRight > scoreLeft)
            setTurretRightAngle();

        
    }

    private void setTurretLeftAngle(){
    }

    private void setTurretCenterAngle(){
    }

    private void setTurretRightAngle(){
    }

    private double scoreFunction(double position, double distance){
        return 100 * costFunction(position) / distance;
    }

    private double costFunction(double position){
        double value = -Math.pow(1 / 550D * position, 6) + 10;
        return Math.max(value, 0);
    }

    public boolean isFinished(){
        return true;
    }
}
