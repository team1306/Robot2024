package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.*;

/**
 * Move the {@link Turret} position to an angle offset from the curret turret's position. 
 * @see {@link MoveTurretToRelativeAngle} 
 * <p>
 * {@link MoveTurretToAbsoluteAngle}
 */
public class MoveTurretByAngle extends Command{

    private final Turret turret;
    private final Rotation2d angleDifference; 

    /**
     * Initialize a new instance of the {@link MoveTurretToRelativeAngle} class
     * @param turret the instance of the turret
     * @param angleDifference any angle between -360 and 360 degrees
     */
    public MoveTurretByAngle(Turret turret, Rotation2d angleDifference){
        this.turret = turret;
        this.angleDifference = angleDifference;

        addRequirements(turret);

        assert Math.abs(angleDifference.getDegrees()) <= 360;
    }

    @Override
    public void initialize(){
        if(angleDifference.getDegrees() % 360 == 0) return;

        double scoreLeft = scoreFunction(
            turret.getCurrentAngle().getDegrees() + angleDifference.getDegrees() - 360, 
            Math.abs(angleDifference.getDegrees() - 360));

        double scoreCenter = scoreFunction(
            turret.getCurrentAngle().getDegrees() + angleDifference.getDegrees(), 
            Math.abs(angleDifference.getDegrees()));

        double scoreRight = scoreFunction(
            turret.getCurrentAngle().getDegrees() + angleDifference.getDegrees() + 360, 
            Math.abs(angleDifference.getDegrees() + 360));


        if(scoreLeft > scoreCenter && scoreLeft > scoreRight)
            setTurretLeftAngle();
        else if(scoreCenter > scoreLeft && scoreCenter > scoreRight)
            setTurretCenterAngle();
        else if(scoreRight > scoreCenter && scoreRight > scoreLeft)
            setTurretRightAngle();
        
        //Ensures it chooses the more central offset if scores are the same
        else if (scoreLeft == scoreCenter){
            if(turret.getCurrentAngle().getDegrees() < 0) 
                setTurretCenterAngle();
            else
                setTurretLeftAngle();
        }
        else if (scoreCenter == scoreRight){
            if(turret.getCurrentAngle().getDegrees() < 0) 
                setTurretRightAngle();
            else
                setTurretCenterAngle();        
        }
        //Case should never happen, but if it does, then it should be reproducible 
        else
            throw new IllegalArgumentException("MoveTurretByAngle failed to calculate valid offset position.\nValues are: " + 
            "\nScoreLeft: "+ scoreLeft + " ScoreCenter: "+ scoreCenter + " ScoreRight: "+ scoreRight +
            "\nAngle Difference: " + angleDifference.getDegrees() + " Turret Angle: " + turret.getCurrentAngle());        
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    private void setTurretLeftAngle(){
        turret.setAngleDifference(angleDifference.minus(Rotation2d.fromDegrees(360)));
    }

    private void setTurretCenterAngle(){
        turret.setAngleDifference(angleDifference);
    }

    private void setTurretRightAngle(){
        turret.setAngleDifference(angleDifference.plus(Rotation2d.fromDegrees(360)));
    }
}
