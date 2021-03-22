package com.example.akulatest
//-----------------------------//
import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
//-----------------------------//
import kotlin.math.pow
import kotlin.math.sqrt
//-----------------------------//
class Joystick @JvmOverloads constructor(
        tContext: Context,
        tAttr: AttributeSet? = null,
        tStyle: Int = 0
) : View(tContext, tAttr, tStyle) {
    override fun onSizeChanged(tNewWidth: Int, tNewHeight: Int, tOldWidth: Int, tOldHeight: Int) {
        super.onSizeChanged(tNewWidth, tNewHeight, tOldWidth, tOldHeight)

        mCenterX            = (width / 2).toFloat()
        mCenterY            = (height / 2).toFloat()

        mHatX               = mCenterX
        mHatY               = mCenterY

        mBaseRadius         = (width / 5).toFloat()
        mHatRadius          = (width / 7).toFloat()

        mMaxMoveRadius      = mBaseRadius
    }

    override fun onDraw(tCanvas: Canvas?) {
        super.onDraw(tCanvas)

        tCanvas?.drawColor(Color.BLACK)

        mColors.setARGB(255, 0, 0, 100)
        tCanvas?.drawCircle(mCenterX, mCenterY, mBaseRadius, mColors)

        if (mMoving) {
            mColors.setARGB(255, 0, 0, 255)
        } else {
            mColors.setARGB(255, 0, 0, 200)
        }

        tCanvas?.drawCircle(mHatX, mHatY, mHatRadius, mColors)
    }

    override fun onTouchEvent(tEvent: MotionEvent): Boolean {
        when (tEvent.action) {
            MotionEvent.ACTION_DOWN -> {
                if (sqrt((tEvent.x - mHatX).pow(2) + (tEvent.y - mHatY).pow(2)) < mHatRadius) {
                    mMoving = true

                    mDeltaX = mHatX - tEvent.x
                    mDeltaY = mHatY - tEvent.y
                }
            }
            MotionEvent.ACTION_MOVE -> {
                if (mMoving) {
                    mRadius = sqrt((tEvent.x + mDeltaX - mCenterX).pow(2) + (tEvent.y + mDeltaY - mCenterY).pow(2))

                    if (mOutOfBounds) {
                        mTouchRadius = sqrt((tEvent.x - mCenterX).pow(2) + (tEvent.y - mCenterY).pow(2))

                        if (mTouchRadius > mMaxMoveRadius) {
                            mHatX = (tEvent.x + mDeltaX - mCenterX) / mRadius * mMaxMoveRadius + mCenterX
                            mHatY = (tEvent.y + mDeltaY - mCenterY) / mRadius * mMaxMoveRadius + mCenterY
                        } else {
                            mDeltaX = mHatX - tEvent.x
                            mDeltaY = mHatY - tEvent.y

                            mHatX = tEvent.x + mDeltaX
                            mHatY = tEvent.y + mDeltaY

                            mOutOfBounds = false
                        }
                    } else {
                        if (mRadius > mMaxMoveRadius) {
                            mHatX = (tEvent.x + mDeltaX - mCenterX) / mRadius * mMaxMoveRadius + mCenterX
                            mHatY = (tEvent.y + mDeltaY - mCenterY) / mRadius * mMaxMoveRadius + mCenterY

                            mOutOfBounds = true
                        } else {
                            mHatX = tEvent.x + mDeltaX
                            mHatY = tEvent.y + mDeltaY
                        }
                    }

                    mListener?.onMove(100.0f / mMaxMoveRadius * (mHatX - mCenterX), 100.0f / mMaxMoveRadius * (mHatY - mCenterY), id)
                    invalidate();
                }
            }
            MotionEvent.ACTION_UP -> {
                mMoving = false
                mOutOfBounds = false

                mHatX = mCenterX
                mHatY = mCenterY

                mListener?.onMove(100.0f / mMaxMoveRadius * (mHatX - mCenterX), 100.0f / mMaxMoveRadius * (mHatY - mCenterY), id)
                invalidate();
            }
        }

        return true
    }

    //----------//

    interface OnMoveListener {
        fun onMove(tPosX: Float, tPosY: Float, tID: Int);
    }

    fun setCustomObjectListener(tListener: OnMoveListener) {
        mListener = tListener
    }

    //----------//

    private var mListener:          OnMoveListener?     = null

    //----------//

    private val mColors:            Paint               = Paint()

    private var mCenterX:           Float               = 0.0f
    private var mCenterY:           Float               = 0.0f

    private var mHatX:              Float               = 0.0f
    private var mHatY:              Float               = 0.0f

    private var mBaseRadius:        Float               = 0.0f
    private var mHatRadius:         Float               = 0.0f

    //----------//

    private var mMoving:            Boolean             = false

    private var mDeltaX:            Float               = 0.0f
    private var mDeltaY:            Float               = 0.0f

    private var mMaxMoveRadius:     Float               = 0.0f
    private var mRadius:            Float               = 0.0f
    private var mTouchRadius:       Float               = 0.0f
    private var mOutOfBounds:       Boolean             = false
}