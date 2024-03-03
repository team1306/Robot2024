package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.MotorUtil;

public class ToggleShooterCommand extends Command{
    private final Shooter shooter;
    private final DoubleSupplier speedSupplier;
    private final DoubleSupplier angleSupplier;
    private double shooterSpeedAngleRatio = 22;

    public ToggleShooterCommand(DoubleSupplier speedSupplier, DoubleSupplier angleSupplier, Shooter shooter){
        addRequirements(shooter);
        this.shooter = shooter;
        this.speedSupplier = speedSupplier;
        this.angleSupplier = angleSupplier;
        SmartDashboard.putNumber("Shooter Speed Angle Ratio", shooterSpeedAngleRatio);
    }
    @Override
    public void initialize(){
        shooter.setTargetSpeed(MotorUtil.clampPercent(speedSupplier.getAsDouble() * (angleSupplier.getAsDouble() / shooterSpeedAngleRatio))); 
    }

    @Override
    public void execute(){
        shooterSpeedAngleRatio = SmartDashboard.getNumber("Shooter Speed Angle Ratio", shooterSpeedAngleRatio);
    }
    @Override
    public void end(boolean interrupted){
        shooter.setTargetSpeed(0);
    }
}
