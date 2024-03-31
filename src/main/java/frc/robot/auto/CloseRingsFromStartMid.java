package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

public class CloseRingsFromStartMid extends ParallelCommandGroup {
    public CloseRingsFromStartMid(NoteDetector detector, Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        // final ToggleShooterCommand shooterCommand = new ToggleShooterCommand(() -> .79, shooter);
        addCommands(
            new InstantCommand(() -> shooter.setTargetSpeed(.79)), //spin up shooter
            new SequentialCommandGroup(
                arm.getPitchControlCommand(driveTrain),
                // .andThen(Commands.waitUntil(arm::atSetpoint)), //aim
                new WaitCommand(1),
                new IntakeIndexCommand(intake), //fire
                new InstantCommand(() -> shooter.setTargetSpeed(0)), //spin up shooter
                AutoCommands.getStartMidToClose1(intake, shooter, arm, driveTrain), //collect close 1 and shoot
                AutoCommands.getClose1ToClose2(intake, shooter, arm, driveTrain), //collect close 2 and shoot
                AutoCommands.getClose2ToClose3(intake, shooter, arm, driveTrain) //collect close 3 and shoot
            )
        );
    }
    
    @Override
    public String getName() {
        return "Close Rings from Start Mid";
    }
}
