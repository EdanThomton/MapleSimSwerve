@file:Suppress("unused")

package team.vaevictis.telemetry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Quaternion
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTable
import kotlin.reflect.KProperty

/**
 * NetworkTables table entry
 */
class NetworkEntryKT (
    table: NetworkTable,
    entryName: String
) {

    /** Entry as a Long */
    var long = NetworkLongEntryKT(table, entryName)
    /** Entry as an array of Longs */
    var longArray = NetworkLongArrayEntryKT(table, entryName)

    /** Entry as a Float */
    var float = NetworkFloatEntryKT(table, entryName)
    /** Entry as an array of Floats */
    var floatArray = NetworkFloatArrayEntryKT(table, entryName)

    /** Entry as a Double */
    var double = NetworkDoubleEntryKT(table, entryName)
    /** Entry as an array of Doubles */
    var doubleArray = NetworkDoubleArrayEntryKT(table, entryName)

    /** Entry as a Boolean */
    var boolean = NetworkBooleanEntryKT(table, entryName)
    /** Entry as an array of Booleans */
    var booleanArray = NetworkBooleanArrayEntryKT(table, entryName)
    /** Entry as a Pose2d */
    var pose2d = NetworkPose2dEntryKT(table, entryName)

    /** Entry as a Pose3d */
    var pose3d = NetworkPose3dEntryKT(table, entryName)

    /** Entry as a Translation2d */
    var translation2d = NetworkTranslation2dEntryKT(table, entryName)

    /** Entry as a Translation3d */
    var translation3d = NetworkTranslation3dEntryKT(table, entryName)

    /** Entry as an array of Translation2ds */
    var translation2dArray = NetworkTranslation2dArrayEntryKT(table, entryName)

    /** Entry as an array of Translation3ds */
    var translation3dArray = NetworkTranslation3dArrayEntryKT(table, entryName)

    /** Entry as a Quaternion */
    var quaternion = NetworkQuaternionEntryKT(table, entryName)

    /** Entry as a single SwerveModuleState */
    var swerveModuleState = NetworkSwerveModuleStateEntryKT(table, entryName)

    /** Entry as an array of SwerveModuleStates */
    var swerveModuleStateArray = NetworkSwerveModuleStateArrayEntryKT(table, entryName)
}

class NetworkLongEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getIntegerTopic(entryName).getEntry(0)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Long = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Long) = entry.set(value)
}
class NetworkLongArrayEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getIntegerArrayTopic(entryName).getEntry(LongArray(0))

    operator fun getValue(thisRef: Any?, property: KProperty<*>): LongArray = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: LongArray) = entry.set(value)
}

class NetworkFloatEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getFloatTopic(entryName).getEntry(0.0f)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Float = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Float) = entry.set(value)
}
class NetworkFloatArrayEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getFloatArrayTopic(entryName).getEntry(FloatArray(0))

    operator fun getValue(thisRef: Any?, property: KProperty<*>): FloatArray = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: FloatArray) = entry.set(value)
}

class NetworkDoubleEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getDoubleTopic(entryName).getEntry(0.0)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Double = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) = entry.set(value)
}
class NetworkDoubleArrayEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getDoubleArrayTopic(entryName).getEntry(DoubleArray(0))

    operator fun getValue(thisRef: Any?, property: KProperty<*>): DoubleArray = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: DoubleArray) = entry.set(value)
}

class NetworkBooleanEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getBooleanTopic(entryName).getEntry(false)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Boolean) = entry.set(value)
}
class NetworkBooleanArrayEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getBooleanArrayTopic(entryName).getEntry(BooleanArray(0))

    operator fun getValue(thisRef: Any?, property: KProperty<*>): BooleanArray = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: BooleanArray) = entry.set(value)
}

class NetworkPose2dEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructTopic(entryName, Pose2d.struct).getEntry(Pose2d.kZero)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Pose2d = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Pose2d) = entry.set(value)
}
class NetworkPose3dEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructTopic(entryName, Pose3d.struct).getEntry(Pose3d.kZero)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Pose3d = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Pose3d) = entry.set(value)
}

class NetworkTranslation2dEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructTopic(entryName, Translation2d.struct).getEntry(Translation2d.kZero)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Translation2d = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Translation2d) = entry.set(value)
}
class NetworkTranslation3dEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructTopic(entryName, Translation3d.struct).getEntry(Translation3d.kZero)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Translation3d = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Translation3d) = entry.set(value)
}

class NetworkTranslation2dArrayEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructArrayTopic(entryName, Translation2d.struct).getEntry(emptyArray<Translation2d>())

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Array<Translation2d> = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Array<Translation2d>) = entry.set(value)
}
class NetworkTranslation3dArrayEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructArrayTopic(entryName, Translation3d.struct).getEntry(emptyArray<Translation3d>())

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Array<Translation3d> = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Array<Translation3d>) = entry.set(value)
}

class NetworkQuaternionEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructTopic(entryName, Quaternion.struct).getEntry(Quaternion())

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Quaternion = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Quaternion) = entry.set(value)
}

class NetworkSwerveModuleStateEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructTopic(entryName, SwerveModuleState.struct).getEntry(SwerveModuleState())

    operator fun getValue(thisRef: Any?, property: KProperty<*>): SwerveModuleState = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: SwerveModuleState) = entry.set(value)
}
class NetworkSwerveModuleStateArrayEntryKT(
    table: NetworkTable,
    entryName: String
) {
    private val entry = table.getStructArrayTopic(entryName, SwerveModuleState.struct).getEntry(emptyArray<SwerveModuleState>())

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Array<SwerveModuleState> = entry.get()
    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Array<SwerveModuleState>) = entry.set(value)
}