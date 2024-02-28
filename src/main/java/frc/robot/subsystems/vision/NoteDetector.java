package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;

public interface NoteDetector {
    public enum State {
        PRESENT,
        NOT_PRESENT,
        UNKNOWN
    }

    /**
     * Gets state of board with respect to notes present on the field.
     * @return Array of length 5, 0 index is top of board
     */
    State[] getNoteState();

    /**
     * Switches command based off of presence of a note
     * @param noteIndex note to switch on
     * @param ifPresent command to run if note present or unknown
     * @param ifNotPresent command to run if note not present
     * @return selected command
     */
    Command notePresenceCommandSwitcher(int noteIndex, Command ifPresent, Command ifNotPresent);
}
