package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToSetpointCommand extends Command {
    final Arm.Setpoint setpoint;
    final Arm arm;
    final BooleanSupplier interruptionFlag;
    
    public MoveArmToSetpointCommand(Arm arm, Arm.Setpoint setpoint, BooleanSupplier interruptionFlag) {
        this.setpoint = setpoint;
        this.arm = arm;
        this.interruptionFlag = interruptionFlag;
        addRequirements(arm);
    }

    public MoveArmToSetpointCommand(Arm arm, Arm.Setpoint setpoint) {
        this(arm, setpoint, null);
    }

    @Override
    public void initialize() {
        arm.setTargetAngle(Rotation2d.fromDegrees(setpoint.getPos()));
    }

    @Override
    public boolean isFinished() {
        return interruptionFlag != null && interruptionFlag.getAsBoolean();
    }
}
