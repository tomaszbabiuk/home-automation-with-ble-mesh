package uitester.vm

import NextionRenderer
import RendereableNVM
import RequestType

/***
 * [UI selection] [State selection] [Instance Id Low] [Instance Id High] [State slot no]
 */
class StateSelectedNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {
    override fun checkMatch(data: ByteArray): Boolean {
        return data.size == 5 &&
                data[0] == RequestType.UISelection.dataByte &&
                data[1] == EntityType.StateSelection.dataByte
    }

    override fun control(data:ByteArray) {
        val instanceIdLow = data[2]
        val instanceIdHigh = data[3]
        val instanceId = instanceIdHigh*256 + instanceIdLow
        val stateSlotNo = data[4]
        println("State selected, instance id=$instanceId, state slot no=$stateSlotNo")
        renderer.render("page control")
    }
}