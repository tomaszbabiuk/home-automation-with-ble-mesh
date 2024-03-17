package uitester.vm

import NextionRenderer
import RendereableNVM
import RequestType

/***
 * [UI selection] [Value selection] [Instance Id LSB] [Instance Id MSB] [Value LSB] [Value ...] [Value ...] [Value MSB]
 */
class ControllerSelectedNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {
    override fun checkMatch(data: ByteArray): Boolean {
        return data.size == 8 &&
                data[0] == RequestType.UISelection.dataByte &&
                data[1] == EntityType.ValueSelection.dataByte
    }

    override fun control(data:ByteArray) {
        val instanceIdLSB = data[2]
        val instanceIdMSB = data[3]
        val instanceId = instanceIdMSB*256 + instanceIdLSB
        val valueLSB = data[4]
        val valueB2 = data[5]
        val valueB3 = data[6]
        val valueMSB = data[7]
        val value = valueMSB* 16777216 + valueB3*65536 + valueB2*256 + valueLSB
        println("Value selected, instance id=$instanceId, value=$value")
        renderer.render("page control")
    }
}