package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * Move the {@link Turret} position to an absolute angle from the curret turret's position. 
 * Meaning that it will move to the actual angle given.
 * @see {@link MoveTurretByAngle} 
 * <p>
 * {@link MoveTurretToRelativeAngle}
 */
public class MoveTurretToAbsoluteAngle extends Command{

    private final Turret turret;
    private final Rotation2d targetAngle; 

    /**
     * Initialize a new instance of the {@link MoveTurretToRelativeAngle} class
     * @param turret the instance of the turret
     * @param targetAngle any angle between -800 and 800 degrees
     */
    public MoveTurretToAbsoluteAngle(Turret turret, Rotation2d targetAngle){
        this.turret = turret;
        this.targetAngle = targetAngle;
        
        assert Math.abs(targetAngle.getDegrees()) <= 800;
    }

    @Override
    public void initialize(){
        turret.setTargetAngle(targetAngle);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
