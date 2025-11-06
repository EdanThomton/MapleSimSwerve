package team.vaevictis.subsystems.swerve.ioa.sim

import edu.wpi.first.math.geometry.Rotation2d
import org.ironmaple.simulation.drivesims.GyroSimulation
import team.vaevictis.subsystems.swerve.ioa.GyroIOA

class GyroSimIOA(
    private val sim: GyroSimulation
): GyroIOA {

    // for zeroing heading
    private var offset = Rotation2d(0.0)

    override val heading: Rotation2d
        get() = sim.gyroReading - offset

    override fun zero() {
        offset = sim.gyroReading
    }

}