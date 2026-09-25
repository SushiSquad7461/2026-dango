package frc.robot.subsystems.shooter;

interface HoodIO {
    void setPosition(double positionInDegrees);
    double getPosition();
    void stepHood(double degrees);
    void zeroHood();
}
