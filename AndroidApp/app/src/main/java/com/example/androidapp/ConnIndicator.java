package com.example.androidapp;
//-----------------------------//
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;
import androidx.annotation.Nullable;
//-----------------------------//
/**
 * Class represents status of some kind (virtual LED)
 * In this app it's used to indicate connection to the server (green LED),
 * stm32 connection (yellow LED) and ROS recording state (red LED)
 */
public class ConnIndicator extends View {
    public ConnIndicator(Context tContext) {
        super(tContext);
        initColors();
    }
    public ConnIndicator(Context tContext, @Nullable AttributeSet tAttr) {
        super(tContext, tAttr);
        initColors();
    }
    public ConnIndicator(Context tContext, @Nullable AttributeSet tAttr, int tStyle) {
        super(tContext, tAttr, tStyle);
        initColors();
    }

    //----------//

    @Override
    protected void onDraw(Canvas tCanvas) {
        super.onDraw(tCanvas);

        tCanvas.drawColor(Color.BLACK);

        mColors.setARGB(255, mBackgroundColor[0], mBackgroundColor[1], mBackgroundColor[0]);
        tCanvas.drawCircle((float)getWidth() / 2, (float)getHeight() / 2, 50.0f, mColors);

        if (mActive) {
            mColors.setARGB(255, mEnableColor[0], mEnableColor[1], mEnableColor[2]);
        } else {
            mColors.setARGB(255, mDisableColor[0], mDisableColor[1], mDisableColor[2]);
        }

        tCanvas.drawCircle((float)getWidth() / 2, (float)getHeight() / 2, 30.0f, mColors);
    }

    //----------//

    /**
     * Setting default colors
     */
    private void initColors() {
        mEnableColor[0] = 0;
        mEnableColor[1] = 200;
        mEnableColor[2] = 0;

        mDisableColor[0] = 0;
        mDisableColor[1] = 100;
        mDisableColor[2] = 0;

        mBackgroundColor[0] = 0;
        mBackgroundColor[1] = 50;
        mBackgroundColor[2] = 0;
    }

    //----------//

    /**
     * Changing state leads to setting active/inactive status of the indicator
     * @param tState Activate the indicator
     */
    void setEnableState(boolean tState) {
        mActive = tState;
        invalidate();
    }

    /**
     * Set color for the active state of the indicator
     * @param tRed Red component
     * @param tGreen Green component
     * @param tBlue Blue component
     */
    void setEnableColor(int tRed, int tGreen, int tBlue) {
        mEnableColor[0] = tRed;
        mEnableColor[1] = tGreen;
        mEnableColor[2] = tBlue;
    }
    /**
     * Set color for the inactive state of the indicator
     * @param tRed Red component
     * @param tGreen Green component
     * @param tBlue Blue component
     */
    void setDisableColor(int tRed, int tGreen, int tBlue) {
        mDisableColor[0] = tRed;
        mDisableColor[1] = tGreen;
        mDisableColor[2] = tBlue;
    }
    /**
     * Set color for the background of the indicator
     * @param tRed Red component
     * @param tGreen Green component
     * @param tBlue Blue component
     */
    void setBackgroundColor(int tRed, int tGreen, int tBlue) {
        mBackgroundColor[0] = tRed;
        mBackgroundColor[1] = tGreen;
        mBackgroundColor[2] = tBlue;
    }

    //----------//

    private final Paint     mColors             = new Paint();
    private boolean         mActive             = false;

    private final int[]     mEnableColor        = new int[3];
    private final int[]     mDisableColor       = new int[3];
    private final int[]     mBackgroundColor    = new int[3];
}