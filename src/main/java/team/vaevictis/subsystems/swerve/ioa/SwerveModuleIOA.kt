package team.vaevictis.subsystems.swerve.ioa

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState

interface SwerveModuleIOA {

    val position: SwerveModulePosition
    var state: SwerveModuleState

}