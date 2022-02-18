package com.example.androidapp;
//-----------------------------//
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import androidx.annotation.Nullable;
import java.lang.Math;
//-----------------------------//
/**
 * Custom joystick that cannot be moved outside of the circle to avoid
 * touching other controls
 */
class Joystick extends View {
    public Joystick(Context tContext) {
        super(tContext);
    }
    public Joystick(Context tContext, @Nullable AttributeSet AttributeSet) {
        super(tContext, AttributeSet);
    }
    public Joystick(Context tContext, @Nullable AttributeSet AttributeSet, int tStyle) {
        super(tContext, AttributeSet, tStyle);
    }

    //----------//

    @Override
    protected void onSizeChanged(int tNewWidth, int tNewHeight, int tOldWidth, int tOldHeight) {
        super.onSizeChanged(tNewWidth, tNewHeight, tOldWidth, tOldHeight);

        mCenterX            = (float)getWidth() / 2;
        mCenterY            = (float)getHeight() / 2;

        mHatX               = mCenterX;
        mHatY               = mCenterY;

        mBaseRadius         = (float)getWidth() / 5;
        mHatRadius          = (float)getHeight() / 7;

        mMaxMoveRadius      = mBaseRadius;
    }

    @Override
    protected void onDraw(Canvas tCanvas) {
        super.onDraw(tCanvas);

        tCanvas.drawColor(Color.BLACK);

        mColors.setARGB(255, 0, 0, 100);
        tCanvas.drawCircle(mCenterX, mCenterY, mBaseRadius, mColors);

        if (mMoving) {
            mColors.setARGB(255, 0, 0, 255);
        } else {
            mColors.setARGB(255, 0, 0, 200);
        }

        tCanvas.drawCircle(mHatX, mHatY, mHatRadius, mColors);
    }

    @Override
    public boolean onTouchEvent(MotionEvent tEvent) {
        switch (tEvent.getAction()) {
            case MotionEvent.ACTION_DOWN:
                if (Math.sqrt(Math.pow((tEvent.getX() - mHatX), 2) + Math.pow(tEvent.getY() - mHatY, 2)) < mHatRadius) {
                    mMoving = true;

                    mDeltaX = mHatX - tEvent.getX();
                    mDeltaY = mHatY - tEvent.getY();
                }

                break;
            case MotionEvent.ACTION_MOVE:
                if (mMoving) {
                    float Radius = (float)Math.sqrt(Math.pow(tEvent.getX() + mDeltaX - mCenterX, 2) + Math.pow(tEvent.getY() + mDeltaY - mCenterY, 2));

                    if (mOutOfBounds) {
                        float TouchRadius = (float)Math.sqrt(Math.pow(tEvent.getX() - mCenterX, 2) + Math.pow(tEvent.getY() - mCenterY, 2));

                        if (TouchRadius > mMaxMoveRadius) {
                            mHatX = (tEvent.getX() + mDeltaX - mCenterX) / Radius * mMaxMoveRadius + mCenterX;
                            mHatY = (tEvent.getY() + mDeltaY - mCenterY) / Radius * mMaxMoveRadius + mCenterY;
                        } else {
                            mDeltaX = mHatX - tEvent.getX();
                            mDeltaY = mHatY - tEvent.getY();

                            mHatX = tEvent.getX() + mDeltaX;
                            mHatY = tEvent.getY() + mDeltaY;

                            mOutOfBounds = false;
                        }
                    } else {
                        if (Radius > mMaxMoveRadius) {
                            mHatX = (tEvent.getX() + mDeltaX - mCenterX) / Radius * mMaxMoveRadius + mCenterX;
                            mHatY = (tEvent.getY() + mDeltaY - mCenterY) / Radius * mMaxMoveRadius + mCenterY;

                            mOutOfBounds = true;
                        } else {
                            mHatX = tEvent.getX() + mDeltaX;
                            mHatY = tEvent.getY() + mDeltaY;
                        }
                    }

                    mListener.onMove(100.0f / mMaxMoveRadius * (mHatX - mCenterX), 100.0f / mMaxMoveRadius * (mHatY - mCenterY), getId());
                    invalidate();
                }

                break;
            case MotionEvent.ACTION_UP:
                mMoving = false;
                mOutOfBounds = false;

                mHatX = mCenterX;
                mHatY = mCenterY;

                mListener.onMove(100.0f / mMaxMoveRadius * (mHatX - mCenterX), 100.0f / mMaxMoveRadius * (mHatY - mCenterY), getId());
                invalidate();

                break;
        }

        return true;
    }

    //----------//

    interface OnMoveListener {
        void onMove(float tPosX, float tPosY, int tID);
    }

    void setCustomObjectListener(OnMoveListener tListener) {
        mListener = tListener;
    }

    //----------//

    private OnMoveListener  mListener               = null;

    //----------//

    private final Paint     mColors                 = new Paint();

    private float           mCenterX                = 0.0f;
    private float           mCenterY                = 0.0f;

    private float           mHatX                   = 0.0f;
    private float           mHatY                   = 0.0f;

    private float           mBaseRadius             = 0.0f;
    private float           mHatRadius              = 0.0f;

    //----------//

    private boolean         mMoving                 = false;

    private float           mDeltaX                 = 0.0f;
    private float           mDeltaY                 = 0.0f;

    private float           mMaxMoveRadius          = 0.0f;
    private boolean         mOutOfBounds            = false;
}