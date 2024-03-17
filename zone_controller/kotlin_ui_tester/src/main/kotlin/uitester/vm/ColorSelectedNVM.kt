package uitester.vm

import NextionRenderer
import RendereableNVM
import RequestType

/***
 * [UI selection] [Color selection] [Instance Id LSB] [Instance Id MSB] [Value HUE Lsb] [Value Hue Msb] [Value Brightness]
 */
class ColorSelectedNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {
    override fun checkMatch(data: ByteArray): Boolean {
        return data.size == 7 &&
                data[0] == RequestType.UISelection.dataByte &&
                data[1] == EntityType.ColorSelection.dataByte
    }

    override fun control(data:ByteArray) {
        val instanceIdLSB = data[2]
        val instanceIdMSB = data[3]
        val instanceId = instanceIdMSB*256 + instanceIdLSB
        val valueHueLSB = data[4]
        val valueHueMSB = data[5]
        val valueHue = valueHueMSB*256 + valueHueLSB
        val valueBrightness = data[6]
        println("Color selected, instance id=$instanceId, hue=$valueHue, brightness=$valueBrightness")
        renderer.render("page control")
    }
}