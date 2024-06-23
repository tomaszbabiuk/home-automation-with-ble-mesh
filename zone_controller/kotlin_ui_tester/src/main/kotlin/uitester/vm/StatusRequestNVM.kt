package uitester.vm

import NextionRenderer
import RendereableNVM
import UIRequestType
import java.util.*

class StatusRequestNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {

    private var count=0

    override fun checkMatch(data: ByteArray): Boolean {
        return data.size == 2 && data[0] == RequestType.UIRequest.dataByte && data[1] == UIRequestType.Status.dataByte
    }

    override fun control(data:ByteArray) {
        val status = count % 4
        val now= Calendar.getInstance()
        val nowSeconds = now.get(Calendar.HOUR_OF_DAY)*3600 + now.get(Calendar.MINUTE)*60 + now.get(Calendar.SECOND)

        renderer.render("radioStatus=$status")
        renderer.render("nowSeconds=$nowSeconds")
    }
}