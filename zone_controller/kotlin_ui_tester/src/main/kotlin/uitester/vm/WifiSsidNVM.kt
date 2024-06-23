package uitester.vm

import NextionRenderer
import RendereableNVM
import UIResponseType

class WifiSsidNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {
    override fun checkMatch(data: ByteArray): Boolean {
        return data.size > 2 && data[0] == RequestType.UIResponse.dataByte && data[1] == UIResponseType.SSIDSelection.dataByte
    }

    override fun control(data: ByteArray) {
        val ssid = String(data.drop(2).toByteArray())
        println("Selected ssid=$ssid")
        renderer.render("keyboard.modeVar.val=0")
        renderer.render("page keyboard")
    }
}