package com.example.akulatest
//-----------------------------//
import android.content.Context
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Message
import android.view.inputmethod.InputMethodManager
import android.widget.Button
import android.widget.EditText
import androidx.appcompat.app.AppCompatActivity
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer
//-----------------------------//
enum class MsgUDP {
    REQUEST_CONN,
    JOYSTICK_COORDS,
    PING,
    STATUS,
    INVALID;

    companion object {
        fun fromInt(tVal: Int): MsgUDP {
            when (tVal) {
                0 -> return REQUEST_CONN
                1 -> return JOYSTICK_COORDS
                2 -> return PING
                3 -> return STATUS
            }

            return INVALID
        }
    }
}
//-----------------------------//
class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val myThread = ThreadUDP(32)
        myThread.start()
        val Joy = findViewById<Joystick>(R.id.joystick)
        val IpLine = findViewById<EditText>(R.id.ipText)
        val PortLine = findViewById<EditText>(R.id.portText)
        val RequestButton = findViewById<Button>(R.id.requestButton)

        val SliderPanel = findViewById<SlidePanel>(R.id.sliderPanel)

        RequestButton.setOnClickListener { tView ->
            IpLine.clearFocus()
            PortLine.clearFocus()

            val imm = getSystemService(Context.INPUT_METHOD_SERVICE) as? InputMethodManager
            imm?.hideSoftInputFromWindow(tView.windowToken, 0)

            if (!IpLine.text.isEmpty() && !PortLine.text.isEmpty()) {                                       //---Add validator---//
                myThread.sendConnectionRequest(IpLine.text.toString(), PortLine.text.toString().toInt())
                SliderPanel.smoothSlideTo(0.0f)
            }
        }

        SliderPanel.setCustomObjectListener(object : SlidePanel.OnSlideListener {
            override fun onSlide() {
                IpLine.clearFocus()
                PortLine.clearFocus()

                SliderPanel.closeKeyboard()
            }
        })

        Joy.setCustomObjectListener(object : Joystick.OnMoveListener {
            override fun onMove(tPosX: Float, tPosY: Float, tID: Int) {
                if (myThread.mCoordsObtainState) {                                                          //---Fix zero coords---//
                    myThread.joystickMoved(tPosX, tPosY)
                }
            }
        })
    }
}

class ThreadUDP(tPacketSize: Int) : HandlerThread("UDP") {
    private var mHandler:       Handler?            = null
    private var mSocket:        DatagramSocket      = DatagramSocket()
    var mCoordsObtainState:     Boolean             = true

    private var mIP:            InetAddress         = InetAddress.getByName("127.0.0.1")
    private var mPort:          Int                 = 50000

    private var mPacketSize:    Int                 = tPacketSize

    override fun onLooperPrepared() {
        super.onLooperPrepared()
        mHandler = object : Handler(looper) {
            override fun handleMessage(tMsg: Message) {
                super.handleMessage(tMsg)
                when (MsgUDP.fromInt(tMsg.what)) {
                    MsgUDP.JOYSTICK_COORDS -> {
                        val Bund = tMsg.data
                        val Floats = Bund.getFloatArray("Coords")
                        val Buff: ByteBuffer = ByteBuffer.allocate(mPacketSize)

                        Floats?.get(0)?.let { Buff.putFloat(it) }
                        Floats?.get(1)?.let { Buff.putFloat(it) }

                        val Packet = DatagramPacket(Buff.array(), mPacketSize, mIP, mPort)

                        mSocket.send(Packet)
                        mCoordsObtainState = false
                        Thread.sleep(20)
                        mCoordsObtainState = true
                    }
                    MsgUDP.REQUEST_CONN -> {
                        val Buff: ByteBuffer = ByteBuffer.allocate(mPacketSize)
                        Buff.putInt(MsgUDP.REQUEST_CONN.ordinal)

                        val Packet = DatagramPacket(Buff.array(), mPacketSize, mIP, mPort)
                        mSocket.send(Packet)
                    }
                    else -> {}
                }
            }
        }
    }

    fun joystickMoved(tPosX: Float, tPosY: Float) {
        if (mHandler != null) {
            val Msg = mHandler?.obtainMessage()

            if (Msg != null) {
                val Bund = Bundle(1)
                Bund.putFloatArray("Coords", floatArrayOf(tPosX, tPosY))
                Msg.data = Bund
                Msg.what = MsgUDP.JOYSTICK_COORDS.ordinal

                mHandler?.sendMessage(Msg)
            }
        }
    }

    fun sendConnectionRequest(tIP: String, tPort: Int) {
        if (mHandler != null) {
            val Msg = mHandler?.obtainMessage()

            if (Msg != null) {
                mIP = InetAddress.getByName(tIP)
                mPort = tPort

                Msg.what = MsgUDP.REQUEST_CONN.ordinal

                mHandler?.sendMessage(Msg)
            }
        }
    }
}
