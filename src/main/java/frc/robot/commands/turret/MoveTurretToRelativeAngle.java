package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.*;

/**
 * Move the {@link Turret} position to an relative angle from the curret turret's position. 
 * Meaning that it will move to the best scoring multiple of the target angle
 * @see {@link MoveTurretByAngle} 
 * <p>
 * {@link MoveTurretToAbsoluteAngle}
 */
public class MoveTurretToRelativeAngle extends Command{ 

    private final Turret turret;
    private final Rotation2d targetAngle;

    /**
     * Initialize a new instance of the {@link MoveTurretToRelativeAngle} class
     * @param turret the instance of the turret
     * @param targetAngle any angle between 0 and 360 degrees
     */
    public MoveTurretToRelativeAngle(Turret turret, Rotation2d targetAngle){
        this.turret = turret;
        this.targetAngle = targetAngle;

        addRequirements(turret);

        assert targetAngle.getDegrees() <= 360 && targetAngle.getDegrees() >= 0;
    }

    @Override
    public void initialize(){        
        
        double[] potentialAngles = new double[6];
        for(int positionIndex = -3; positionIndex < 3; positionIndex++)
            potentialAngles[positionIndex + 3] = targetAngle.getDegrees() + 360 * positionIndex;
        
        double bestScore = 0;
        double bestScoreAngle = 0;
        for(double angle : potentialAngles){
            double score = scoreFunction(angle, Math.abs(angle - turret.getCurrentAngle().getDegrees()));
            if(score > bestScore){
                bestScore = score;
                bestScoreAngle = angle;
            }
        }

        if(bestScore == 0){ 
            throw new IllegalArgumentException("MoveTurretToRelativeAngle could not find a valid angle to move to.\n Values: "
        + "\nTarget Angle: " + targetAngle.getDegrees());}

        turret.setTargetAngle(Rotation2d.fromDegrees(bestScoreAngle));
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
