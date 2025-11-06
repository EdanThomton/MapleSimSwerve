package team.vaevictis.telemetry

import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import java.util.function.Supplier

class SwerveStatesSendable(
    initialSwerveStates: Array<SwerveModuleState>
): Sendable {

    var swerveStates = initialSwerveStates

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("SwerveDrive")

        builder.addDoubleProperty("Front Left Angle", {swerveStates[0].angle.radians}, null)
        builder.addDoubleProperty("Front Left Velocity", {swerveStates[0].speedMetersPerSecond}, null)

        builder.addDoubleProperty("Front Right Angle", {swerveStates[1].angle.radians}, null)
        builder.addDoubleProperty("Front Right Velocity", {swerveStates[1].speedMetersPerSecond}, null)

        builder.addDoubleProperty("Back Left Angle", {swerveStates[2].angle.radians}, null)
        builder.addDoubleProperty("Back Left Velocity", {swerveStates[2].speedMetersPerSecond}, null)

        builder.addDoubleProperty("Back Right Angle", {swerveStates[3].angle.radians}, null)
        builder.addDoubleProperty("Back Right Velocity", {swerveStates[3].speedMetersPerSecond}, null)

    }

    fun updateSwerveStates(states: Array<SwerveModuleState>) {
        swerveStates = states
    }

}