package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

@FunctionalInterface
public interface AutonomousFactory {
    Command createAutonomousCommand(NoteDetector detector, DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake);
}
