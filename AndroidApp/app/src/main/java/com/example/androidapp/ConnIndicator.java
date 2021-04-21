package com.example.androidapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.Nullable;

public class ConnIndicator extends View {
    public ConnIndicator(Context tContext) {
        super(tContext);
    }
    public ConnIndicator(Context tContext, @Nullable AttributeSet tAttr) {
        super(tContext, tAttr);
    }
    public ConnIndicator(Context tContext, @Nullable AttributeSet tAttr, int tStyle) {
        super(tContext, tAttr, tStyle);
    }

    //----------//

    @Override
    protected void onDraw(Canvas tCanvas) {
        super.onDraw(tCanvas);

        tCanvas.drawColor(Color.BLACK);

        mColors.setARGB(255, 0, 50, 0);
        tCanvas.drawCircle((float)getWidth() / 2, (float)getHeight() / 2, 50.0f, mColors);

        if (mConnected) {
            mColors.setARGB(255, 0, 200, 0);
        } else {
            mColors.setARGB(255, 0, 100, 0);
        }

        tCanvas.drawCircle((float)getWidth() / 2, (float)getHeight() / 2, 30.0f, mColors);
    }

    //----------//

    void setConnectionState(boolean tState) {
        mConnected = tState;
        invalidate();
    }

    //----------//

    private Paint           mColors             = new Paint();
    private boolean         mConnected          = false;
}