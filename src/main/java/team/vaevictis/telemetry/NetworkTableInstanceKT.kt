package team.vaevictis.telemetry

import edu.wpi.first.networktables.NetworkTableInstance

/**
 * A NetworkTables instance
 */
class NetworkTableInstanceKT (
    private val instance: NetworkTableInstance
) {

    companion object {
        /**
         * Default NetworkTables instance
         */
        val DEFAULT: NetworkTableInstanceKT = NetworkTableInstanceKT(NetworkTableInstance.getDefault())
    }

    operator fun get(table: String): NetworkTableKT = NetworkTableKT(
        instance.getTable(table)
    )

    operator fun get(table: String, entry: String): NetworkEntryKT = NetworkEntryKT(
        instance.getTable(table), entry
    )

}