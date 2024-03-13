package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmCommand extends Command {
    private final Arm arm;
    public static double speed = 0.42;
    private Rotation2d targetAngle;
    private final DoubleSupplier rotationSupplier;

    public MoveArmCommand(Arm arm, DoubleSupplier rotationSupplier){
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;
        addRequirements(arm);

        SmartDashboard.putNumber("Arm Manual Speed", speed);
    }

    @Override
    public void initialize(){
        arm.setControlMode(Arm.ControlMode.AUTOMATIC);
        targetAngle = arm.getTargetAngle();
    }

    @Override
    public void execute(){
        speed = SmartDashboard.getNumber("Arm Manual Speed", speed);
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