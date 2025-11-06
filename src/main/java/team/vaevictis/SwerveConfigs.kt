package team.vaevictis

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import team.vaevictis.subsystems.swerve.SwerveConfig
import team.vaevictis.subsystems.swerve.SwerveDriverConfig
import team.vaevictis.subsystems.swerve.SwerveModuleConfig

const val WHEEL_POS = 0.33655
val swerveDriveConfig = SwerveConfig(

    SwerveModuleConfig(
        4, 3,
        0.5, 0.0, 0.0,
        1.75, 0.0, 0.05,
        0.0, 0.0789, 0.0,
        0.0,
        Translation2d(WHEEL_POS, WHEEL_POS)
    ),
    SwerveModuleConfig(
        6, 5,
        0.5, 0.0, 0.0,
        1.75, 0.0, 0.05,
        0.0, 0.0789, 0.0,
        0.0,
        Translation2d(WHEEL_POS, -WHEEL_POS)
    ),
    SwerveModuleConfig(
        2, 1,
        0.5, 0.0, 0.0,
        1.75, 0.0, 0.05,
        0.0, 0.0789, 0.0,
        0.0,
        Translation2d(-WHEEL_POS, WHEEL_POS)
    ),
    SwerveModuleConfig(
        8, 7,
        0.5, 0.0, 0.0,
        1.75, 0.0, 0.05,
        0.0, 0.0789, 0.0,
        0.0,
        Translation2d(-WHEEL_POS, -WHEEL_POS)
    ),
    SwerveDriverConfig(
        maxAccel = 2.5,
        maxSpeedMPS = 4.8,
        maxAngularAccel = .075,
        maxAngularSpeed = 2.0 * Math.PI,

        fieldCentric = true,

        initialPose = Pose2d(
            Translation2d(7.122, 2.026),
            Rotation2d.k180deg
        )
    )
)