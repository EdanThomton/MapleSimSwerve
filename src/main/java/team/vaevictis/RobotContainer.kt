package team.vaevictis

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import gay.zharel.botlin.commands.CoroutineCommand
import gay.zharel.botlin.units.inch
import gay.zharel.botlin.units.inches
import gay.zharel.botlin.units.kilo
import gay.zharel.botlin.units.kilograms
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import team.vaevictis.subsystems.swerve.SwerveConfig
import team.vaevictis.subsystems.swerve.SwerveDrive
import team.vaevictis.subsystems.swerve.SwerveModuleConfig
import team.vaevictis.subsystems.swerve.ioa.SwerveModuleIOA
import team.vaevictis.subsystems.swerve.ioa.sim.GyroSimIOA
import team.vaevictis.subsystems.swerve.ioa.sim.SwerveModuleSimIOA

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
    val swerveSimConfig: DriveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        .withGyro(COTS.ofNav2X())
        .withRobotMass(55.kilograms)
        .withSwerveModule(COTS.ofMAXSwerve(
            DCMotor.getNEO(1),
            DCMotor.getNeo550(1),
            COTS.WHEELS.BLUE_NITRILE_TREAD.cof,
            2
        ))
        .withTrackLengthTrackWidth(26.5.inches, 26.5.inches)
        .withBumperSize(30.0.inches, 30.0.inches)

    val swerveDriveSim = SwerveDriveSimulation(
        swerveSimConfig,
        swerveDriveConfig.driverConfig.initialPose
    )

    val moduleFL = SwerveModuleSimIOA(
        swerveDriveSim.modules[0],
        swerveDriveConfig.frontLeft
    )
    val moduleFR = SwerveModuleSimIOA(
        swerveDriveSim.modules[1],
        swerveDriveConfig.frontRight
    )
    val moduleBL = SwerveModuleSimIOA(
        swerveDriveSim.modules[2],
        swerveDriveConfig.backLeft
    )
    val moduleBR = SwerveModuleSimIOA(
        swerveDriveSim.modules[3],
        swerveDriveConfig.backRight
    )
    val gyro = GyroSimIOA(swerveDriveSim.gyroSimulation)

    val swerveDrive = SwerveDrive(
        moduleFL,
        moduleFR,
        moduleBL,
        moduleBR,
        gyro,
        swerveDriveConfig
    )

    val leftJoystick = CommandJoystick(0)
    val rightJoystick = CommandJoystick(1)

    init {
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSim);

        configureBindings()
    }

    /** Use this method to define your `trigger->command` mappings. */
    private fun configureBindings() {
        swerveDrive.defaultCommand = Commands.run({
            swerveDrive.drive(
                -leftJoystick.x,
                leftJoystick.y,
                -rightJoystick.x
            )
        }, swerveDrive)
    }

    fun getAutonomousCommand(): Command? {
        // TODO: Implement properly
        return null
    }

    fun updateTelemetry() {
        Telemetry.updateSwerveStates(swerveDrive.states)
        Telemetry.updateTargetSwerveStates(swerveDrive.targetStates)
        Telemetry.updatePose(swerveDrive.pose)
        Telemetry.updateSimulatedPose(swerveDriveSim.simulatedDriveTrainPose)
    }
}