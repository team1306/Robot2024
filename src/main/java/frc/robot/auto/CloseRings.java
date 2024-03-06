package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

public class CloseRings extends SequentialCommandGroup {
    public CloseRings(NoteDetector detector, Intake intake, Shooter shooter, Arm arm) {
        addCommands(
            AutoCommands.getClose1StartMid(intake)
        );
    }
}
