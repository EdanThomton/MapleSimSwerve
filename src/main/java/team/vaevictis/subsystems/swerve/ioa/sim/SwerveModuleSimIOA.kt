package team.vaevictis.subsystems.swerve.ioa.sim

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.SubsystemBase
import gay.zharel.botlin.units.amps
import gay.zharel.botlin.units.radians
import gay.zharel.botlin.units.radiansPerSecond
import gay.zharel.botlin.units.to
import gay.zharel.botlin.units.volts
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import team.vaevictis.subsystems.swerve.SwerveModuleConfig
import team.vaevictis.subsystems.swerve.ioa.SwerveModuleIOA
import team.vaevictis.subsystems.swerve.WheelConstants

class SwerveModuleSimIOA(
    private val swerveModuleSim: SwerveModuleSimulation,
    config: SwerveModuleConfig
): SwerveModuleIOA, SubsystemBase() {

    private val driveMotor = swerveModuleSim
        .useGenericMotorControllerForDrive()
        .withCurrentLimit(50.amps)

    private val steerMotor = swerveModuleSim
        .useGenericControllerForSteer()
        .withCurrentLimit(20.amps)

    private val driveController = PIDController(config.driveP, config.driveI, config.driveD)
    private val steerController = PIDController(config.steerP, config.steerI, config.steerD)

    private val driveFeedforward = SimpleMotorFeedforward(config.ks, config.kv, config.ka)

    private var driveFFVolts = 0.0

    init {
        steerController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            swerveModuleSim.driveWheelFinalPosition.radians * WheelConstants.WHEEL_RADIUS_METERS,
            swerveModuleSim.steerAbsoluteFacing
        )

    override var state: SwerveModuleState
        get() = SwerveModuleState(
            swerveModuleSim.driveWheelFinalSpeed.radiansPerSecond * WheelConstants.WHEEL_RADIUS_METERS,
            swerveModuleSim.steerAbsoluteFacing
        )
        set(value) {
            val currentAngle = swerveModuleSim.steerAbsoluteFacing
            value.optimize(currentAngle)
            value.cosineScale(currentAngle)
            setWheelPosition(value.speedMetersPerSecond, value.angle.radians)
        }

    override fun simulationPeriodic() {
        val driveAngularVelocity = swerveModuleSim.driveWheelFinalSpeed.to(RadiansPerSecond)
        val steerAbsoluteFacing = swerveModuleSim.steerAbsoluteFacing.radians

        val driveAppliedVolts = driveFFVolts + driveController.calculate(driveAngularVelocity)
        val steerAppliedVolts = steerController.calculate(steerAbsoluteFacing)

        driveMotor.requestVoltage(driveAppliedVolts.volts)
        steerMotor.requestVoltage(steerAppliedVolts.volts)
    }

    private fun setWheelPosition(speed: Double, angle: Double) {
        driveFFVolts = driveFeedforward.calculate(speed)
        driveController.setpoint = speed
        steerController.setpoint = angle
    }

}