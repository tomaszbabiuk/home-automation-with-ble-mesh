package uitester.vm

import NextionRenderer
import RendereableNVM
import UIResponseType

class NtpAddressNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {
    override fun checkMatch(data: ByteArray): Boolean {
        return data.size > 2 && data[0] == RequestType.UIResponse.dataByte && data[1] == UIResponseType.NtpPassword.dataByte
    }

    override fun control(data: ByteArray) {
        val ntpAddress = String(data.drop(2).toByteArray())
        println("Typed NTP address=$ntpAddress")
        renderer.render("page setupSummary")
    }
}