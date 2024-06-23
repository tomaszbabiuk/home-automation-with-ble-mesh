enum class RequestType(val dataByte: Byte) {
    PageLoaded(0x00.toByte()),
    UIRequest(0x01.toByte()),
    UIResponse(0x02.toByte())
}

enum class Page(val dataByte: Byte) {
    WiFiScanning(0x00.toByte()),
    ChooseSSID(0x01.toByte()),
}

enum class UIRequestType(val dataByte: Byte) {
    Status(0x00.toByte()),
}

enum class UIResponseType(val dataByte: Byte) {
    SSIDSelection(0x00.toByte()),
    WiFiPassword(0x01.toByte()),
    NtpPassword(0x02.toByte())
}
