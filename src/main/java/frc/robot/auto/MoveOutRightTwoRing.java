package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveOutRightTwoRing extends ParallelCommandGroup {
    public MoveOutRightTwoRing(DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) {
        addRequirements(intake);
        System.out.println("Running Auto");
        final ToggleShooterCommand shooterCommand = new ToggleShooterCommand(() -> 1, arm.getCurrentAngle()::getDegrees, shooter);
        final Timer timer = new Timer();
        addCommands( //all commands run at once
            shooterCommand, //turns on shooter
            new SequentialCommandGroup( //following commands run in sequence
                new InstantCommand(driveTrain.gyro::reset),
                new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.SHOOT_CLOSE, () -> true), //aim
                new WaitCommand(1.5),
                new IntakeIndexCommand(intake), //fire
                new InstantCommand(shooterCommand::stop), //turn off shooter
                new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE, () -> true), //arm down
                new ParallelDeadlineGroup(new WaitCommand(.8), //drive out
                    driveTrain.driveBySetpointPercentagesCommand(0, 0.3)
                ),
                new InstantCommand(() -> {
                    intake.setTargetSpeed(0.74);
                    timer.restart();
                }),
                new ParallelDeadlineGroup(new WaitCommand(2.7),
                    driveTrain.driveBySetpointPercentagesCommand(Math.max(0, 0.3), Math.max(0, 0.3))
                ).until(intake::notePresent),
                new InstantCommand(()-> {
                    intake.setTargetSpeed(0);
                    timer.stop();
                })
                )
            );
        
    }

    @Override
    public String getName() {
        return "Move Out Right Two Ring";
    }   
}
