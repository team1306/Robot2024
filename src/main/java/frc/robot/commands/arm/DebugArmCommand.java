package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.DashboardGetter;

public class DebugArmCommand extends Command {
    private final Arm arm;
    private double minAngle = 0;
    private double maxAngle = 110;
    private double targetAngle = 0;

    public DebugArmCommand(Arm arm){
        this.arm = arm;
        this.addRequirements(arm);

        DashboardGetter.addGetDoubleData("Arm Min Angle", minAngle, value -> minAngle = value);
        DashboardGetter.addGetDoubleData("Arm Max Angle", maxAngle, value -> maxAngle = value);
        DashboardGetter.addGetDoubleData("Arm Target Angle", targetAngle, value -> targetAngle = value);
    }

    @Override
    public void execute(){
        arm.setTargetAngle(Rotation2d.fromDegrees(targetAngle));
    }

    public void reset() {
        final Rotation2d armCurrentAngle = arm.getCurrentAngle();
        arm.setTargetAngle(armCurrentAngle);
        targetAngle = armCurrentAngle.getDegrees();
    }
}