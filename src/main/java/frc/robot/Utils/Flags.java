package frc.robot.Utils;

public class Flags {
    public static enum subsystemsStates {
        noPiece, holdingPiece, loadedPiece
    }

    public static enum buttonMapStates {
        endgameMode, notEndgameMode
    }

    public static subsystemsStates pieceState = subsystemsStates.noPiece;

    public static buttonMapStates buttonMapMode = buttonMapStates.notEndgameMode;
}
