package uitester.vm

import NextionRenderer
import RendereableNVM
import UIResponseType

class WifiPasswordNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {
    override fun checkMatch(data: ByteArray): Boolean {
        return data.size > 2 && data[0] == RequestType.UIResponse.dataByte && data[1] == UIResponseType.WiFiPassword.dataByte
    }

    override fun control(data: ByteArray) {
        val password = String(data.drop(2).toByteArray())
        println("Typed password=$password")
        renderer.render("keyboard.modeVar.val=1")
        renderer.render("page keyboard")
    }
}