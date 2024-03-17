package uitester.vm

import NextionRenderer
import RendereableNVM

class InboxNVM(renderer: NextionRenderer) : RendereableNVM(renderer) {
    override fun checkMatch(data: ByteArray): Boolean {
        return data.size == 3 && data[0] == RequestType.Data.dataByte && data[1] == EntityType.InboxSubjects.dataByte
    }

    override fun control(data: ByteArray) {
        val page = data[2]
        if (page == 0x00.toByte()) {
            renderer.render("vis slot0Btn,1")
            renderer.render("slot0Btn.txt=\"Thank you for choosing Automate Everything\"")
            renderer.render("slot0Btn.font=2")

            renderer.render("vis slot1Btn,1")
            renderer.render("slot1Btn.txt=\"Automation enabled\"")
            renderer.render("slot1Btn.font=2")

            renderer.render("vis slot2Btn,1")
            renderer.render("slot2Btn.txt=\"Automation disabled\"")
            renderer.render("slot2Btn.font=1")

            renderer.render("vis slot3Btn,1")
            renderer.render("slot3Btn.txt=\"A problem with sensor one\"")
            renderer.render("slot3Btn.font=1")

            renderer.render("vis slot4Btn,1")
            renderer.render("slot4Btn.txt=\"A problem with sensor two\"")
            renderer.render("slot4Btn.font=1")

            renderer.render("vis slot5Btn,1")
            renderer.render("slot5Btn.txt=\"A problem with sensor three\"")
            renderer.render("slot5Btn.font=1")

            renderer.render("vis slot6Btn,1")
            renderer.render("slot6Btn.txt=\"System is DOWN\"")
            renderer.render("slot6Btn.font=1")

            renderer.render("vis slot7Btn,1")
            renderer.render("slot7Btn.txt=\"System is UP\"")
            renderer.render("slot7Btn.font=1")

            renderer.render("vis prevPageBtn,0")
            renderer.render("vis nextPageBtn,1")
            renderer.render("vis loadingBtn,0")
        } else {
            renderer.render("vis slot0Btn,1")
            renderer.render("slot0Btn.txt=\"Page 2, message 1\"")
            renderer.render("slot0Btn.font=2")

            renderer.render("vis slot1Btn,1")
            renderer.render("slot1Btn.txt=\"Page 2, message 2\"")
            renderer.render("slot1Btn.font=2")

            renderer.render("vis slot2Btn,0")
            renderer.render("vis slot3Btn,0")
            renderer.render("vis slot4Btn,0")
            renderer.render("vis slot5Btn,0")
            renderer.render("vis slot6Btn,0")
            renderer.render("vis slot7Btn,0")

            renderer.render("vis prevPageBtn,1")
            renderer.render("vis nextPageBtn,0")
            renderer.render("vis loadingBtn,0")
        }
    }
}