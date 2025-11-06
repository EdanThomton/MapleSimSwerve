package team.vaevictis.subsystems.swerve.ioa

import edu.wpi.first.math.geometry.Rotation2d

interface GyroIOA {

    val heading: Rotation2d

    fun zero()

}