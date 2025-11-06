package team.vaevictis.subsystems.swerve

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj2.command.SubsystemBase
import team.vaevictis.Telemetry
import team.vaevictis.subsystems.swerve.ioa.GyroIOA
import team.vaevictis.subsystems.swerve.ioa.SwerveModuleIOA
import java.util.concurrent.locks.ReentrantLock

class SwerveDrive(
    val moduleFL: SwerveModuleIOA,
    val moduleFR: SwerveModuleIOA,
    val moduleBL: SwerveModuleIOA,
    val moduleBR: SwerveModuleIOA,
    val gyro: GyroIOA,
    val config: SwerveConfig
): SubsystemBase() {

    private val kinematics = config.kinematics

    private val swerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.kZero,
        positions,
        config.driverConfig.initialPose
    )

    private val odometryNotifier = Notifier(this::updateOdometry)
    private val odometryLock = ReentrantLock()

    private var isFieldCentric = config.driverConfig.fieldCentric

    private var currentChassisSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)

    var targetStates = emptyArray<SwerveModuleState>()

    val currentSpeeds: ChassisSpeeds
        get() = currentChassisSpeeds

    val yaw: Rotation2d
        get() = gyro.heading

    val states: Array<SwerveModuleState>
        get() = arrayOf(
            moduleFL.state,
            moduleFR.state,
            moduleBL.state,
            moduleBR.state
        )
    val positions: Array<SwerveModulePosition>
        get() = arrayOf(
            moduleFL.position,
            moduleFR.position,
            moduleBL.position,
            moduleBR.position
        )
    val pose: Pose2d
        get() {
            odometryLock.lock()
            val estPose = swerveDrivePoseEstimator.estimatedPosition
            odometryLock.unlock()
            return estPose
        }

    init {
        odometryNotifier.startPeriodic(0.01)
    }

    fun updateOdometry() {
        odometryLock.lock()
        try {
            swerveDrivePoseEstimator.update(gyro.heading, positions)
        } finally {
            odometryLock.unlock()
        }
    }

    fun resetOdometry(pose: Pose2d) {
        driveChassisSpeeds(ChassisSpeeds(0.0, 0.0, 0.0))

        odometryLock.lock()
        try {
            swerveDrivePoseEstimator.resetPosition(gyro.heading, positions, pose)
        } finally {
            odometryLock.unlock()
        }
    }

    fun drive(x: Double, y: Double, rot: Double) {
        val metersPerSecondX = x * config.driverConfig.maxSpeedMPS
        val metersPerSecondY = y * config.driverConfig.maxSpeedMPS
        val rotationSpeed = rot * config.driverConfig.maxAngularSpeed

        if(isFieldCentric) {
            // NWU from blue alliance
            currentChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                metersPerSecondX,
                metersPerSecondY,
                rotationSpeed,
                yaw
            )
        } else {
            currentChassisSpeeds = ChassisSpeeds(metersPerSecondX, metersPerSecondY, rotationSpeed)
        }

        driveChassisSpeeds(currentChassisSpeeds)
    }

    fun driveChassisSpeeds(speeds: ChassisSpeeds) {

        currentChassisSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        val wheelStates = kinematics.toWheelSpeeds(currentChassisSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(
            wheelStates,
            config.driverConfig.maxAccel
        )

        targetStates = wheelStates

        moduleFL.state = wheelStates[0]
        moduleFR.state = wheelStates[1]
        moduleBL.state = wheelStates[2]
        moduleBR.state = wheelStates[3]
    }

}