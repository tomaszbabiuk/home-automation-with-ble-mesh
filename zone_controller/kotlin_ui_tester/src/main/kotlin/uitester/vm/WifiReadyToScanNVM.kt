package uitester.vm

import NextionRenderer
import RendereableNVM

class WifiReadyToScanNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {

    private var success = false

    override fun checkMatch(data: ByteArray): Boolean {
        return data.size == 2 && data[0] == RequestType.PageLoaded.dataByte && data[1] == Page.WiFiScanning.dataByte
    }

    override fun control(data:ByteArray) {
        if (success) {
            renderer.render("page wifiSsid")
        } else {
            renderer.render("page noNetworks")
        }
        success = !success
    }
}