package com.example.akulatest

import android.content.Context
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.InputMethodManager
import androidx.annotation.Px
import androidx.core.view.ViewCompat
import androidx.customview.widget.ViewDragHelper
import kotlin.math.abs

class SlidePanel @JvmOverloads constructor(
        tContext: Context,
        tAttr: AttributeSet? = null,
        tStyle: Int = 0
) : ViewGroup(tContext, tAttr, tStyle) {
    private val mDragHelper: ViewDragHelper
    private lateinit var mOpenSlider: View
    private lateinit var mSlidingPanel: View
    private var mInitialMotionY:            Float                   = 0.0f
    private var mDragRange:                 Int                     = 0
    private var mTop:                       Int                     = 0
    private var mDragOffset:                Float                   = 0.0f

    private var mListener:                  OnSlideListener?        = null

    override fun onFinishInflate() {
        super.onFinishInflate()
        mOpenSlider = getChildAt(0)
        mSlidingPanel = getChildAt(1)
    }

    fun smoothSlideTo(slideOffset: Float): Boolean {
        val topBound = paddingTop
        val y = (topBound + slideOffset * mDragRange).toInt()

        if (mDragHelper.smoothSlideViewTo(mOpenSlider, mOpenSlider.left, y)) {
            ViewCompat.postInvalidateOnAnimation(this)
            return true
        }

        return false
    }

    private inner class DragHelperCallback : ViewDragHelper.Callback() {
        override fun tryCaptureView(child: View, pointerId: Int): Boolean {
            return child === mOpenSlider
        }

        override fun onViewPositionChanged(changedView: View, left: Int, top: Int, @Px dx: Int, @Px dy: Int) {
            mTop = top
            mDragOffset = top.toFloat() / mDragRange

            requestLayout()
        }

        override fun onViewReleased(releasedChild: View, xvel: Float, yvel: Float) {                            //---This shit needs to be fixed---//
            var top = paddingTop

            if (yvel > 0 || yvel == 0f && mDragOffset > 0.5f) {
                top += mDragRange
            }

            mDragHelper.settleCapturedViewAt(releasedChild.left, top)
        }

        override fun getViewVerticalDragRange(child: View): Int {
            return mDragRange
        }

        override fun clampViewPositionVertical(tChild: View, tTop: Int, tDeltaY: Int): Int {
            val TopBound = paddingTop
            val BottomBound: Int = mSlidingPanel.height + mSlidingPanel.paddingTop

            return minOf(maxOf(tTop, TopBound), BottomBound)
        }
    }

    override fun computeScroll() {
        if (mDragHelper.continueSettling(true)) {
            ViewCompat.postInvalidateOnAnimation(this)
        }
    }

    override fun onInterceptTouchEvent(ev: MotionEvent): Boolean {
        val action = ev.action
        if (action != MotionEvent.ACTION_DOWN) {
            mDragHelper.cancel()
            return super.onInterceptTouchEvent(ev)
        }
        if (action == MotionEvent.ACTION_CANCEL || action == MotionEvent.ACTION_UP) {
            mDragHelper.cancel()
            return false
        }

        val y = ev.y
        var interceptTap = false

        when (action) {
            MotionEvent.ACTION_DOWN -> {
                mInitialMotionY = y
                interceptTap = mDragHelper.isViewUnder(mOpenSlider, x.toInt(), y.toInt())
            }
            MotionEvent.ACTION_MOVE -> {
                val ady = abs(y - mInitialMotionY)
                val slop = mDragHelper.touchSlop

                if (ady > slop) {
                    mDragHelper.cancel()
                    return false
                }
            }
        }

        return mDragHelper.shouldInterceptTouchEvent(ev) || interceptTap
    }

    override fun onTouchEvent(ev: MotionEvent): Boolean {
        mDragHelper.processTouchEvent(ev)

        mListener?.onSlide()

        val action = ev.action
        val y = ev.y
        val isHeaderViewUnder = mDragHelper.isViewUnder(mOpenSlider, x.toInt(), y.toInt())

        when (action) {
            MotionEvent.ACTION_DOWN -> {
                mInitialMotionY = y
            }
            MotionEvent.ACTION_UP -> {
                val dy = y - mInitialMotionY
                val slop = mDragHelper.touchSlop

                if (isHeaderViewUnder) {
                    if (dy * dy < slop * slop) {
                        if (mDragOffset == 0f) {
                            smoothSlideTo(1.0f)
                        } else {
                            smoothSlideTo(0.0f)
                        }
                    } else {
                        if (mOpenSlider.y > mSlidingPanel.height / 2) {
                            smoothSlideTo(1.0f)
                        } else {
                            smoothSlideTo(0.0f)
                        }
                    }
                }
            }
        }
        return isHeaderViewUnder && isViewHit(mOpenSlider, x.toInt(), y.toInt()) || isViewHit(mSlidingPanel, x.toInt(), y.toInt())
    }

    private fun isViewHit(view: View?, x: Int, y: Int): Boolean {
        val viewLocation = IntArray(2)
        view?.getLocationOnScreen(viewLocation)
        val parentLocation = IntArray(2)
        getLocationOnScreen(parentLocation)
        val screenX = parentLocation[0] + x
        val screenY = parentLocation[1] + y

        if (view != null) {
            return screenX >= viewLocation[0] && screenX < viewLocation[0] + view.width && screenY >= viewLocation[1] && screenY < viewLocation[1] + view.height
        } else {
            return false
        }
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        measureChildren(widthMeasureSpec, heightMeasureSpec)
        val maxWidth = MeasureSpec.getSize(widthMeasureSpec)
        val maxHeight = MeasureSpec.getSize(heightMeasureSpec)
        setMeasuredDimension(resolveSizeAndState(maxWidth, widthMeasureSpec, 0),
                resolveSizeAndState(maxHeight, heightMeasureSpec, 0))
    }

    override fun onLayout(tChanged: Boolean, tLeft: Int, tTop: Int, tRight: Int, tBottom: Int) {
        mDragRange = mSlidingPanel.measuredHeight
        mOpenSlider.layout(0, mTop, tRight, mTop + mOpenSlider.measuredHeight)
        mSlidingPanel.layout(0, mTop - mSlidingPanel.measuredHeight, tRight, mTop)
    }

    init {
        mDragHelper = ViewDragHelper.create(this, 1.0f, DragHelperCallback())
    }

    //----------//

    interface OnSlideListener {
        fun onSlide();
    }

    fun setCustomObjectListener(tListener: OnSlideListener) {
        mListener = tListener
    }

    fun closeKeyboard() {
        val imm = context.getSystemService(Context.INPUT_METHOD_SERVICE) as InputMethodManager
        imm.hideSoftInputFromWindow(windowToken, 0)
    }
}