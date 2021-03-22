package com.example.akulatest
//-----------------------------//
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
//-----------------------------//
class ConnIndicator @JvmOverloads constructor(
        tContext: Context,
        tAttr: AttributeSet? = null,
        tStyle: Int = 0
) : View(tContext, tAttr, tStyle) {
    override fun onDraw(tCanvas: Canvas?) {
        super.onDraw(tCanvas)

        tCanvas?.drawColor(Color.BLACK)

        mColors.setARGB(255, 0, 50, 0)
        tCanvas?.drawCircle((width / 2).toFloat(), (height / 2).toFloat(), 50.0f, mColors)

        if (mConnected) {
            mColors.setARGB(255, 0, 200, 0)
        } else {
            mColors.setARGB(255, 0, 100, 0)
        }

        tCanvas?.drawCircle((width / 2).toFloat(), (height / 2).toFloat(), 30.0f, mColors)
    }

    fun setConnectionState(tState: Boolean) {
        mConnected = tState
        invalidate()
    }

    //----------//

    private val mColors:        Paint           = Paint()
    private var mConnected:     Boolean         = false
}