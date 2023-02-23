package frc.robot.subsystems.limb;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ClawSubsystem extends Subsystem {
    /** Closes the claw. */
    void grab();
    /** Releases the contents of the claw. */
    void release();
    /** Toggles the state of the claw. */
    void toggleClaw();

    /** Returns true if the claw is closed. */
    boolean getClawClosed();

    /** Spins the left and right motors to the specified speeds. */
    void spinMotors(double leftSpeed, double rightSpeed);
    /** Spins both motors to the specified speed. */
    void spinMotors(double bothSpeed);
    /** Stops the motors. */
    void stopMotors();

    /** Returns the angle, in radians, of the wrist. */
    double getWristAngle();
    /** Sets the target angle of the wrist.
     * @param angle (radians)
     */
    void setWristAngle(double angle);
}
