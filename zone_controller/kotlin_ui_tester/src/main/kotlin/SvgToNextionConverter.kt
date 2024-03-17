import org.apache.batik.transcoder.SVGAbstractTranscoder
import org.apache.batik.transcoder.TranscoderException
import org.apache.batik.transcoder.TranscoderInput
import org.apache.batik.transcoder.TranscoderOutput
import org.apache.batik.transcoder.image.PNGTranscoder
import java.awt.image.BufferedImage
import java.io.*
import javax.imageio.ImageIO

data class Point(val x0: Int, val y0: Int, val x1: Int, val y1: Int)

class SvgToNextionConverter {
    @Throws(TranscoderException::class, IOException::class)

    private fun convertSVGToPNGBuffered(svgFilePath: String, width: Float, height: Float): BufferedImage {
        val initialFile = File(svgFilePath)
        val inputStream: InputStream = FileInputStream(initialFile)

        val resultByteStream = ByteArrayOutputStream()
        val transcoderInput = TranscoderInput(inputStream)
        val transcoderOutput = TranscoderOutput(resultByteStream)
        val pngTranscoder = PNGTranscoder()
        pngTranscoder.addTranscodingHint(SVGAbstractTranscoder.KEY_HEIGHT, width)
        pngTranscoder.addTranscodingHint(SVGAbstractTranscoder.KEY_WIDTH, height)
        pngTranscoder.transcode(transcoderInput, transcoderOutput)
        resultByteStream.flush()

        return ImageIO.read(ByteArrayInputStream(resultByteStream.toByteArray()))
    }

    fun convert(svgFilePath: String, width: Float, height: Float, debug: Boolean = false): List<Point> {
        val result=ArrayList<Point>()
        val image = convertSVGToPNGBuffered(svgFilePath, width, height)
        if (debug) {
            println("Width=${image.width}, Height=${image.height})")
        }
        val maxX = image.width - 1
        val maxY = image.width - 1
        for (y in 0..maxY) {
            var prev = false
            var lineStart=0
            for (x in 0..maxX) {
                val on = image.getRGB(x, y) != 0
                if (on && !prev) {
                    lineStart = x
                }

                if (debug) {
                    print(if (on) "1" else "0")
                }

                if ((!on && prev) || (x==maxX && on)) {
                    if (debug) {
                        print("%")
                    }

                    result.add(Point(x0 = lineStart, y0=y, x1=x, y1=y))
                }

                prev=on
            }

            if (debug) {
                println()
            }
        }

        return result
    }
}