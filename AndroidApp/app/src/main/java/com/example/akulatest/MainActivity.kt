package com.example.akulatest
//-----------------------------//
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Message
import android.widget.Button
import androidx.appcompat.app.AppCompatActivity
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer

//-----------------------------//
class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val myThread = ThreadUDP()
        myThread.start()
        val Joy = findViewById <Joystick>(R.id.joystick)

        Joy.setCustomObjectListener(object : Joystick.OnMoveListener {
            override fun onMove(tPosX: Float, tPosY: Float, tID: Int) {
                if (myThread.mObtainState) {                //---Fix zero coords---//
                    myThread.joystickMoved(tPosX, tPosY)
                }
            }
        })
    }
}

class ThreadUDP : HandlerThread("UDP") {
    private var mHandler: Handler? = null
    private var mSocket: DatagramSocket = DatagramSocket()
    var mObtainState: Boolean = true

    override fun onLooperPrepared() {
        super.onLooperPrepared()
        mHandler = object : Handler(looper) {
            override fun handleMessage(tMsg: Message) {
                super.handleMessage(tMsg)

                val Bund = tMsg.getData()
                val Floats = Bund.getFloatArray("Coords")

                val serverAddr = InetAddress.getByName("192.168.46.255")
                val Buff: ByteBuffer = ByteBuffer.allocate(8)

                Floats?.get(0)?.let { Buff.putFloat(it) }
                Floats?.get(1)?.let { Buff.putFloat(it) }

                val xPos = ByteBuffer.wrap(Buff.array()).getFloat();
                val yPos = ByteBuffer.wrap(Buff.array()).getFloat();

                val dp = DatagramPacket(Buff.array(), 8, serverAddr, 50000)

                mSocket.send(dp)
                mObtainState = false
                println("Here")
                Thread.sleep(20)
                mObtainState = true
            }
        }
    }

    fun joystickMoved(tPosX: Float, tPosY: Float) {
        if (mHandler != null) {
            val Msg = mHandler?.obtainMessage()

            if (Msg != null) {
                val Bund = Bundle(1)
                Bund.putFloatArray("Coords", floatArrayOf(tPosX, tPosY))
                Msg.setData(Bund)

                mHandler?.sendMessage(Msg)
            }
        }
    }
}