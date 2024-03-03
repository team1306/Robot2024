package frc.robot.subsystems.vision;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface NoteDetector {
    public enum State {
        PRESENT,
        NOT_PRESENT,
        UNKNOWN
    }

    public enum Checkpoints {
        TOP(PathPlannerPath.fromPathFile("Shoot-Top to Far-2.path").getStartingDifferentialPose()),
        TOP_MIDDLE(PathPlannerPath.fromPathFile("Scan-Top to Far-3.path").getStartingDifferentialPose()),
        BOTTOM_MIDDLE(PathPlannerPath.fromPathFile("Scan-Bottom to Far-2.path").getStartingDifferentialPose());

        final Pose2d pose;

        private Checkpoints(Pose2d pose) {
            this.pose = pose;
        }
    }

    /**
     * Gets state of board with respect to notes present on the field.
     * @return Array of length 5, 0 index is top of board
     */
    State[] getNoteState();
    
    /**
     * read note position
     * @return command to read note position
     */
    Command read(Checkpoints checkpoint);

    /**
     * Switches command based off of presence of a note
     * @param noteIndex note to switch on
     * @param ifPresent command to run if note present or unknown
     * @param ifNotPresent command to run if note not present
     * @return selected command
     */
    default Command notePresenceCommandSwitcher(int noteIndex, Command ifPresent, Command ifNotPresent) {
        if (noteIndex >= 5) {
            throw new IllegalArgumentException("Note index out of bounds");
        }
        return getNoteState()[noteIndex] == State.NOT_PRESENT ? ifNotPresent : ifPresent;
    }
}