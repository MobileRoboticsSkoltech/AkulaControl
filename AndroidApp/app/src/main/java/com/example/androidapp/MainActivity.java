package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;
import androidx.lifecycle.Lifecycle;
import androidx.lifecycle.LifecycleObserver;
import androidx.lifecycle.OnLifecycleEvent;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.text.InputFilter;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.lang.ref.WeakReference;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class MainActivity extends AppCompatActivity {
    static class HandlerUI extends Handler {
        HandlerUI() {
            super(Looper.getMainLooper());
        }

        @Override
        public void handleMessage(Message tMsg) {
            super.handleMessage(tMsg);

            System.out.println(Header.fromInt(tMsg.what));

            switch (Header.fromInt(tMsg.what)) {
                case PING:
                    if (!mStateServerLED) {
                        mServerLED.get().setConnectionState(true);
                        mStateServerLED = true;
                    }

                    break;
                case DISCONNECTED:
                    if (mStateServerLED) {
                        mServerLED.get().setConnectionState(false);
                        mStateServerLED = false;
                    }

                    if (mStateStm32LED) {
                        mStm32LED.get().setConnectionState(false);
                        mStateStm32LED = false;
                    }

                    break;
                case STM32_ONLINE:
                    if (!mStateStm32LED) {
                        mStm32LED.get().setConnectionState(true);
                        mStateStm32LED = true;
                    }

                    break;
                case STM32_DISCONNECTED:
                    if (mStateStm32LED) {
                        mStm32LED.get().setConnectionState(false);
                        mStateStm32LED = false;
                    }

                    break;
                case LATENCY_RESPONSE:
                    Bundle Bund = tMsg.getData();
                    double Latency = Bund.getDouble("Latency");
                    LatencyText.get().setText(String.valueOf(Latency));

                    System.out.println("Printed latency");

                    break;
                case ENCODER:
                    Bundle EncoderBund = tMsg.getData();
                    double LeftEncoder = EncoderBund.getDouble("LeftEncoder");
                    double RightEncoder = EncoderBund.getDouble("RightEncoder");

                    LeftEncoderText.get().setText(String.valueOf(LeftEncoder));
                    RightEncoderText.get().setText(String.valueOf(RightEncoder));

                    System.out.println("Printed encoders");

                    break;
            }
        }

        void setLED(ConnIndicator tLED) {
            mServerLED = new WeakReference <>(tLED);
        }
        void setStmLED(ConnIndicator tLED) {
            mStm32LED = new WeakReference <>(tLED);
        }

        void setLatencyText(TextView tView) {
            LatencyText = new WeakReference <>(tView);
        }

        void setLeftEncoderText(TextView tView) {
            LeftEncoderText = new WeakReference <>(tView);
        }
        void setRightEncoderText(TextView tView) {
            RightEncoderText = new WeakReference <>(tView);
        }

        //----------//

        private boolean mStateServerLED = false;
        private WeakReference <ConnIndicator> mServerLED;

        private boolean mStateStm32LED = false;
        private WeakReference <ConnIndicator> mStm32LED;

        private WeakReference <TextView> LatencyText;

        private WeakReference <TextView> LeftEncoderText;
        private WeakReference <TextView> RightEncoderText;
    }

    //----------//

    public class MinimizeObserver implements LifecycleObserver {
        @OnLifecycleEvent(Lifecycle.Event.ON_STOP)
        void onAppBackgrounded() {
            mNetworkThread.close();
        }
    }

    //----------//

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getLifecycle().addObserver(new MinimizeObserver());

        InputFilter IpFilter = (tSrc, tSrcStart, tSrcEnd, tDest, tDestStart, tDestEnd) -> {
            String IpRange = "(?:0|[1-9]|[1-9][0-9]|1[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
            Pattern IpPattern = Pattern.compile("^" + IpRange + "(\\.(" + IpRange + "(\\.(" + IpRange + "(\\.(" + IpRange + ")?)?)?)?)?)?$");

            String destTxt = tDest.toString();
            String resultingTxt = destTxt.substring(0, tDestStart) + tSrc.subSequence(tSrcStart, tSrcEnd) + destTxt.substring(tDestEnd);

            Matcher Match = IpPattern.matcher(resultingTxt);

            if (!Match.matches()) {
                return "";
            }

            return null;
        };

        InputFilter PortFilter = (tSrc, tSrcStart, tSrcEnd, tDest, tDestStart, tDestEnd) -> {
            Pattern PortPattern = Pattern.compile("^([1-9]|[1-9][0-9]{1,3}|[1-5][0-9]{4}|6[0-4][0-9]{3}|65[0-4][0-9]{2}|655[0-2][0-9]|6553[0-5])$");

            String destTxt = tDest.toString();
            String resultingTxt = destTxt.substring(0, tDestStart) + tSrc.subSequence(tSrcStart, tSrcEnd) + destTxt.substring(tDestEnd);

            Matcher Match = PortPattern.matcher(resultingTxt);

            if (!Match.matches()) {
                return "";
            }

            return null;
        };

        Joystick Joy = findViewById(R.id.joystick);
        EditText IpLine = findViewById(R.id.ipText);
        EditText PortLine = findViewById(R.id.portText);
        Button RequestButton = findViewById(R.id.requestButton);
        Button LatencyButton = findViewById(R.id.latbutton);

        SlidePanel SliderPanel = findViewById(R.id.sliderPanel);

        IpLine.setFilters(new InputFilter[] {IpFilter});
        PortLine.setFilters(new InputFilter[] {PortFilter});

        ConnIndicator StmIndicator = findViewById(R.id.stm32Indicator);
        StmIndicator.setEnableColor(200, 200, 0);
        StmIndicator.setDisableColor(100, 100, 0);
        StmIndicator.setBackgroundColor(50, 50, 0);

        mTestHandler.setLED(findViewById(R.id.serverIndicator));
        mTestHandler.setStmLED(findViewById(R.id.stm32Indicator));

        mTestHandler.setLatencyText(findViewById(R.id.textView2));

        mTestHandler.setLeftEncoderText(findViewById(R.id.leftEncoder));
        mTestHandler.setRightEncoderText(findViewById(R.id.rightEncoder));

        RequestButton.setOnClickListener(tView -> {
            IpLine.clearFocus();
            PortLine.clearFocus();

            InputMethodManager imm = (InputMethodManager)getSystemService(Context.INPUT_METHOD_SERVICE);
            imm.hideSoftInputFromWindow(tView.getWindowToken(), 0);

            if (!IpLine.getText().toString().matches("") && !PortLine.getText().toString().matches("")) {                                       //---Add validator---//
                try {
                    mNetworkThread.sendConnectionRequest(IpLine.getText().toString(), Integer.parseInt(PortLine.getText().toString()));
                } catch (SocketException | UnknownHostException e) {
                    e.printStackTrace();
                }

                SliderPanel.smoothSlideTo(0.0f);
            }
        });

        SliderPanel.setCustomObjectListener(() -> {
            IpLine.clearFocus();
            PortLine.clearFocus();

            SliderPanel.closeKeyboard();
        });

        Joy.setCustomObjectListener((tPosX, tPosY, tID) -> {
            mNetworkThread.sendJoystickCoords(tPosX, tPosY);
        });

        LatencyButton.setOnClickListener(tView -> {
            mNetworkThread.sendLatencyTest();
        });
    }
    @Override
    protected void onStart() {
        super.onStart();

        mNetworkThread = new NetworkThread(mTestHandler);
        mNetworkThread.start();
    }

    //----------//

    HandlerUI       mTestHandler        = new HandlerUI();
    NetworkThread   mNetworkThread;
}


