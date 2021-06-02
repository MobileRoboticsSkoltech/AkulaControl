package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
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
                    if (!mStateLED) {
                        mLED.get().setConnectionState(true);
                        mStateLED = true;
                    }

                    break;
                case DISCONNECTED:
                    mLED.get().setConnectionState(false);
                    mStateLED = false;

                    break;
                case LATENCY_RESPONSE:
                    Bundle Bund = tMsg.getData();
                    double Latency = Bund.getDouble("Latency");
                    LatencyText.get().setText(String.valueOf(Latency));

                    System.out.println("Printed latency");

                    break;
            }
        }

        void setLED(ConnIndicator tLED) {
            mLED = new WeakReference <>(tLED);
        }
        void setLatencyText(TextView tView) {
            LatencyText = new WeakReference <>(tView);
        }

        //----------//

        private boolean mStateLED = false;
        private WeakReference <ConnIndicator> mLED;
        private WeakReference <TextView> LatencyText;
    }

    //----------//

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

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

        mTestHandler.setLED(findViewById(R.id.connIndicator));
        mTestHandler.setLatencyText(findViewById(R.id.textView2));

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


