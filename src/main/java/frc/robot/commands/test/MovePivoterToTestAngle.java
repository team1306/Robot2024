package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivoter;
import frc.robot.util.DashboardGetter;

public class MovePivoterToTestAngle extends Command{
    private final Pivoter pivoter;
    private double pivoterTestAngle = 0;

    public MovePivoterToTestAngle(Pivoter pivoter){
        this.pivoter = pivoter;
        
        DashboardGetter.addGetDoubleData("Pivoter Test Angle", pivoterTestAngle, value -> pivoterTestAngle = value);
        
        addRequirements(pivoter);
    }

    @Override
    public void execute(){
        pivoter.setTargetAngle(Rotation2d.fromDegrees(pivoterTestAngle));
    }
}
