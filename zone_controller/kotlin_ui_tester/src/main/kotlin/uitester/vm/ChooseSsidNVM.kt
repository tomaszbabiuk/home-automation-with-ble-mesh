package uitester.vm

import NextionRenderer
import RendereableNVM

class ChooseSsidNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {

    override fun checkMatch(data: ByteArray): Boolean {
        return data.size == 2 && data[0] == RequestType.PageLoaded.dataByte && data[1] == Page.ChooseSSID.dataByte
    }

    override fun control(data:ByteArray) {
        renderer.render("scanResult.txt=\"network1;network2;network3\"")
        renderer.render("sys0=-1")
        renderer.render("click downBtn,0")
    }
}