package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int DRIVE_MOTOR_ID;
    public final int ANGLE_MOTOR_ID;
    public final int ABS_ENCIDER_PORT;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
        this.DRIVE_MOTOR_ID = driveMotorID;
        this.ANGLE_MOTOR_ID = angleMotorID;
        this.ABS_ENCIDER_PORT = canCoderID;
        this.angleOffset = angleOffset;
    }
}
