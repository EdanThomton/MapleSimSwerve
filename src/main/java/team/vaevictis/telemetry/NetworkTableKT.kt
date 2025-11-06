package team.vaevictis.telemetry

import edu.wpi.first.networktables.NetworkTable

/**
 * NetworkTables table
 */
class NetworkTableKT (
    private val table: NetworkTable
) {

    operator fun get(entry: String): NetworkEntryKT = NetworkEntryKT(
        table, entry
    )

}