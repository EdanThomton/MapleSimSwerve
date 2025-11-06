package team.vaevictis.subsystems.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import kotlin.math.PI

data class SwerveModuleConfig (
    val speedID: Int, val angleID: Int,
    val driveP: Double, val driveI: Double, val driveD: Double,
    val steerP: Double, val steerI: Double, val steerD: Double,
    val ks: Double, val kv: Double, val ka: Double,
    val chassisAngleOffset: Double,
    val chassisLocation: Translation2d
)

data class SwerveDriverConfig(
    val maxAccel: Double,
    val maxSpeedMPS: Double,
    val maxAngularAccel: Double,
    val maxAngularSpeed: Double,
    val fieldCentric: Boolean,
    val initialPose: Pose2d
)

data class SwerveConfig(
    val frontLeft: SwerveModuleConfig,
    val frontRight: SwerveModuleConfig,
    val backLeft: SwerveModuleConfig,
    val backRight: SwerveModuleConfig,
    val driverConfig: SwerveDriverConfig
) {
    val kinematics: SwerveDriveKinematics get() = SwerveDriveKinematics(
        frontLeft.chassisLocation,
        frontRight.chassisLocation,
        backLeft.chassisLocation,
        backRight.chassisLocation
    )
}

object WheelConstants {

    const val DRIVE_MOTOR_REDUCTION = 5.08
    const val STEER_MOTOR_REDUCTION = 9424.0 / 203.0

    const val WHEEL_RADIUS = 1.5 // in
    const val WHEEL_RADIUS_METERS = WHEEL_RADIUS * 0.0254
    const val WHEEL_TURNS_TO_METERS = WHEEL_RADIUS_METERS * 2.0 * Math.PI

    const val DRIVE_POSITION_CONVERSION_FACTOR = 2.0 * PI / DRIVE_MOTOR_REDUCTION
    const val DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0

    const val STEER_POSITION_CONVERSION_FACTOR = 2.0 * PI
    const val STEER_VELOCITY_CONVERSION_FACTOR = STEER_POSITION_CONVERSION_FACTOR / 60.0

}