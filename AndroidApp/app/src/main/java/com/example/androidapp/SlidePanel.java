package com.example.androidapp;

import android.content.Context;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.inputmethod.InputMethodManager;

import androidx.annotation.NonNull;
import androidx.annotation.Px;
import androidx.core.view.ViewCompat;
import androidx.customview.widget.ViewDragHelper;

public class SlidePanel extends ViewGroup {
    private class DragHelperCallback extends ViewDragHelper.Callback {
        @Override
        public boolean tryCaptureView(@NonNull View child, int pointerId) {
            return child == mOpenSlider;
        }

        @Override
        public void onViewPositionChanged(@NonNull View tChangedView, int tLeft, int tTop, @Px int tdx, @Px int tdy) {
            mTop = tTop;
            mDragOffset = (float)tTop / mDragRange;

            requestLayout();
        }

        @Override
        public void onViewReleased(@NonNull View tReleasedChild, float tXVel, float tYVel) {                            //---This shit needs to be fixed---//
            int top = getPaddingTop();

            if (tYVel > 0 || tYVel == 0f && mDragOffset > 0.5f) {
                top += mDragRange;
            }

            mDragHelper.settleCapturedViewAt(tReleasedChild.getLeft(), top);
        }

        @Override
        public int getViewVerticalDragRange(@NonNull View tChild) {
            return mDragRange;
        }

        @Override
        public int clampViewPositionVertical(@NonNull View tChild, int tTop, int tDeltaY) {
            int TopBound = getPaddingTop();
            int BottomBound = mSlidingPanel.getHeight() + mSlidingPanel.getPaddingTop();

            return Math.min(Math.max(tTop, TopBound), BottomBound);
        }
    }

    //----------//

    public SlidePanel(Context tContext) {
        super(tContext);

        mDragHelper = ViewDragHelper.create(this, 1.0f, new DragHelperCallback());
    }
    public SlidePanel(Context tContext, AttributeSet tAttr) {
        super(tContext, tAttr);

        mDragHelper = ViewDragHelper.create(this, 1.0f, new DragHelperCallback());
    }
    public SlidePanel(Context tContext, AttributeSet tAttr, int tStyle) {
        super(tContext, tAttr, tStyle);

        mDragHelper = ViewDragHelper.create(this, 1.0f, new DragHelperCallback());
    }

    @Override
    protected void onFinishInflate() {
        super.onFinishInflate();

        mOpenSlider = getChildAt(0);
        mSlidingPanel = getChildAt(1);
    }

    @Override
    public void computeScroll() {
        if (mDragHelper.continueSettling(true)) {
            ViewCompat.postInvalidateOnAnimation(this);
        }
    }

    @Override
    public boolean onInterceptTouchEvent(MotionEvent tEvent) {
        int Action = tEvent.getAction();

        if (Action != MotionEvent.ACTION_DOWN) {
            mDragHelper.cancel();
            return super.onInterceptTouchEvent(tEvent);
        }

        if (Action == MotionEvent.ACTION_CANCEL || Action == MotionEvent.ACTION_UP) {
            mDragHelper.cancel();
            return false;
        }

        float y = tEvent.getY();
        boolean interceptTap = false;

        switch (Action) {
            case MotionEvent.ACTION_DOWN:
                mInitialMotionY = y;
                interceptTap = mDragHelper.isViewUnder(mOpenSlider, (int)getX(), (int)getY());

                break;
            case MotionEvent.ACTION_MOVE:
                float ady = Math.abs(y - mInitialMotionY);
                int slop = mDragHelper.getTouchSlop();

                if (ady > slop) {
                    mDragHelper.cancel();
                    return false;
                }

                break;
        }

        return mDragHelper.shouldInterceptTouchEvent(tEvent) || interceptTap;
    }

    @Override
    public boolean onTouchEvent(MotionEvent tEvent) {
        mDragHelper.processTouchEvent(tEvent);

        mListener.onSlide();

        int Action = tEvent.getAction();
        float y = tEvent.getY();
        boolean isHeaderViewUnder = mDragHelper.isViewUnder(mOpenSlider, (int)getX(), (int)y);

        System.out.println(isHeaderViewUnder);

        switch (Action) {
            case MotionEvent.ACTION_DOWN:
                mInitialMotionY = y;

                break;
            case MotionEvent.ACTION_UP:
                float dy = y - mInitialMotionY;
                int slop = mDragHelper.getTouchSlop();

                if (isHeaderViewUnder) {
                    if (dy * dy < slop * slop) {
                        if (mDragOffset == 0f) {
                            smoothSlideTo(1.0f);
                        } else {
                            smoothSlideTo(0.0f);
                        }
                    } else {
                        if (mOpenSlider.getY() > (float)mSlidingPanel.getHeight() / 2) {
                            smoothSlideTo(1.0f);
                        } else {
                            smoothSlideTo(0.0f);
                        }
                    }
                }

                break;
        }

        return isHeaderViewUnder && isViewHit(mOpenSlider, (int)getX(), (int)y) || isViewHit(mSlidingPanel, (int)getX(), (int)y);
    }

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        measureChildren(widthMeasureSpec, heightMeasureSpec);
        int maxWidth = MeasureSpec.getSize(widthMeasureSpec);
        int maxHeight = MeasureSpec.getSize(heightMeasureSpec);
        setMeasuredDimension(
                resolveSizeAndState(maxWidth, widthMeasureSpec, 0),
                resolveSizeAndState(maxHeight, heightMeasureSpec, 0));
    }

    @Override
    protected void onLayout(boolean tChanged, int tLeft, int tTop, int tRight, int tBottom) {
        mDragRange = mSlidingPanel.getMeasuredHeight();
        mOpenSlider.layout(0, mTop, tRight, mTop + mOpenSlider.getMeasuredHeight());
        mSlidingPanel.layout(0, mTop - mSlidingPanel.getMeasuredHeight(), tRight, mTop);
    }

    //----------//

    boolean smoothSlideTo(float slideOffset) {
        int topBound = getPaddingTop();
        int y = topBound + (int)slideOffset * mDragRange;

        if (mDragHelper.smoothSlideViewTo(mOpenSlider, mOpenSlider.getLeft(), y)) {
            ViewCompat.postInvalidateOnAnimation(this);
            return true;
        }

        return false;
    }

    private boolean isViewHit(View view, int x, int y) {
        int[] viewLocation = new int[2];
        view.getLocationOnScreen(viewLocation);
        int[] parentLocation = new int[2];
        getLocationOnScreen(parentLocation);
        int screenX = parentLocation[0] + x;
        int screenY = parentLocation[1] + y;

        return  screenX >= viewLocation[0]                      &&
                screenX < viewLocation[0] + view.getWidth()     &&
                screenY >= viewLocation[1]                      &&
                screenY < viewLocation[1] + view.getHeight();
    }

    void closeKeyboard() {
        InputMethodManager imm = (InputMethodManager)getContext().getSystemService(Context.INPUT_METHOD_SERVICE);
        imm.hideSoftInputFromWindow(getWindowToken(), 0);
    }

    //----------//

    interface OnSlideListener {
        void onSlide();
    }

    void setCustomObjectListener(OnSlideListener tListener) {
        mListener = tListener;
    }

    //----------//

    private OnSlideListener         mListener                       = null;

    //----------//

    private final ViewDragHelper    mDragHelper;
    private View                    mOpenSlider;
    private View                    mSlidingPanel;
    private float                   mInitialMotionY                 = 0.0f;
    private int                     mDragRange                      = 0;
    private int                     mTop                            = 0;
    private float                   mDragOffset                     = 0.0f;
}