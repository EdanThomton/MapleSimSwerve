package team.vaevictis

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import team.vaevictis.telemetry.*

object Telemetry {

    private val table = NetworkTableInstanceKT.DEFAULT["VaeVictis"]

    private var statesNT by table["states"].swerveModuleStateArray
    private var stateTargetsNT by table["stateTargets"].swerveModuleStateArray
    private var poseNT by table["pose"].pose2d
    private var simulatedPoseNT by table["simulatedPose"].pose2d

    fun updateSwerveStates(states: Array<SwerveModuleState>) {
        statesNT = states
    }
    fun updatePose(pose: Pose2d) {
        poseNT = pose
    }
    fun updateSimulatedPose(pose: Pose2d) {
        simulatedPoseNT = pose
    }

    fun updateTargetSwerveStates(states: Array<SwerveModuleState>) {
        stateTargetsNT = states
    }
}