public enum IntakeState {
    IDLE(false, Direction.OFF),
    DEPLOYING(true, Direction.OFF),
    DEPLOYED(true, Direction.OFF),
    ROLLERS_IN(true, Direction.FORWARD),
    ROLLERS_OUT(true, Direction.REVERSE),
    STOWING(false, Direction.OFF),
    STOWED(false, Direction.OFF),
    WIGGLING(true, Direction.FORWARD);

    public final boolean intakeExtended;
    public final Direction direction;

    private IntakeState(boolean extended, Direction direction) {
        this.intakeExtended = extended;
        this.direction = direction;
    }
}
