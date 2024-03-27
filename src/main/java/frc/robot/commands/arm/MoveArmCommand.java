package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.DashboardGetter;

import java.util.function.DoubleSupplier;

public class MoveArmCommand extends Command {
    private final Arm arm;
    public static double speed = 0.42;
    private Rotation2d targetAngle;
    private final DoubleSupplier rotationSupplier;

    public MoveArmCommand(Arm arm, DoubleSupplier rotationSupplier){
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;
        addRequirements(arm);

        DashboardGetter.addGetDoubleData("Arm Manual Speed", speed, value -> speed = value);
    }

    @Override
    public void initialize(){
        targetAngle = arm.getTargetAngle();
    }

    @Override
    public void execute(){
        targetAngle = Rotation2d.fromDegrees(Math.max(0.0, targetAngle.plus(Rotation2d.fromDegrees(rotationSupplier.getAsDouble() * -1 * speed)).getDegrees()));
        SmartDashboard.putNumber("Arm Target Angle", targetAngle.getDegrees());
        arm.setTargetAngle(targetAngle);
    }

    public void reset() {
        final Rotation2d armCurrentAngle = arm.getCurrentAngle();
        arm.setTargetAngle(armCurrentAngle);
        targetAngle = armCurrentAngle;
    }
}