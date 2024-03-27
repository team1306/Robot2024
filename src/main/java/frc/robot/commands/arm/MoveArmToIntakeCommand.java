package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.SetpointOptions;

public class MoveArmToIntakeCommand extends MoveArmToSetpointCommand {
    private BooleanSupplier notePresentFlag;

    public MoveArmToIntakeCommand(Arm arm, BooleanSupplier notePresentFlag, BooleanSupplier interruptionFlag) {
        super(arm, SetpointOptions.INTAKE, interruptionFlag);
        this.notePresentFlag = notePresentFlag;
    }

    @Override
    public void execute() {
        arm.setTargetAngle(Rotation2d.fromDegrees(notePresentFlag.getAsBoolean() ?
            SetpointOptions.DOWN.getPos() :
            SetpointOptions.INTAKE.getPos()
        ));
    }
    
}
