package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

public class CloseRingsFromStartMid extends SequentialCommandGroup {
    public CloseRingsFromStartMid(NoteDetector detector, Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        // final ToggleShooterCommand shooterCommand = new ToggleShooterCommand(() -> .79, shooter);
        addCommands(
            new InstantCommand(() -> {shooter.setTargetSpeed(.79);driveTrain.setSideVoltages(0, 0);}), //spin up shooter
            arm.getPitchControlCommand(driveTrain),
            new WaitUntilCommand(() -> shooter.getTopRPM() > 4300 && arm.atSetpoint()),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(() -> shooter.setTargetSpeed(0)),
            AutoCommands.getStartMidToClose1(intake, shooter, arm, driveTrain), //collect close 1 and shoot
            AutoCommands.getClose1ToClose2(intake, shooter, arm, driveTrain), //collect close 2 and shoot
            AutoCommands.getClose2ToClose3(intake, shooter, arm, driveTrain) //collect close 3 and shoot
        );
    }
    
    @Override
    public String getName() {
        return "Close Rings from Start Mid";
    }
}
