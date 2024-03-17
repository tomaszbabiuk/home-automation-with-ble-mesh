enum class RequestType(val dataByte: Byte) {
    Data(0x00.toByte()),
    UISelection(0x01.toByte())
}

enum class EntityType(val dataByte: Byte) {
    SSIDs(0x00.toByte()),
    SSIDSelection(0x01.toByte()),
    WiFiPassword(0x02.toByte()),
    InboxSubjects(0x03.toByte()),
    InboxBody(0x04.toByte()),
    DevicesPage(0x05.toByte()),
    InterfaceValueOfState(0x06.toByte()),
    StateSelection(0x07.toByte()),
    InterfaceValueOfController(0x08.toByte()),
    ValueSelection(0x09.toByte()),
    InterfaceValueOfColor(0x0A.toByte()),
    ColorSelection(0x0B.toByte()),

}