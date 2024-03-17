package uitester.vm

import NextionRenderer
import RendereableNVM

class WifiSsidNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {
    override fun checkMatch(data: ByteArray): Boolean {
        return data.size > 2 && data[0] == RequestType.UISelection.dataByte && data[1] == EntityType.SSIDSelection.dataByte
    }

    override fun control(data: ByteArray) {
        val ssid = String(data.drop(2).toByteArray())
        println("Selected ssid=$ssid")
        renderer.render("page wifiPassword")
    }
}