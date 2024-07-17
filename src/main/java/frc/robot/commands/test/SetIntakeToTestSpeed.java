package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.util.DashboardGetter;

public class SetIntakeToTestSpeed extends Command{
    private final Intake intake;
    private double speed = 0;
    
    public SetIntakeToTestSpeed(Intake intake) {
        this.intake = intake;

        DashboardGetter.addGetDoubleData("Intake Test Speed", speed, value -> speed = value);    
    }

    @Override
    public void execute() {
        intake.setTargetSpeed(speed);
    }
}
