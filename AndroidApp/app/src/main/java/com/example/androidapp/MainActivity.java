package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import android.text.InputFilter;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;

import java.lang.ref.WeakReference;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class MainActivity extends AppCompatActivity {
    class HandlerUI extends Handler {
        @Override
        public void handleMessage(Message tMsg) {
            super.handleMessage(tMsg);

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
            }
        }

        void setLED(ConnIndicator tLED) {
            mLED = new WeakReference <>(tLED);
        }

        private WeakReference <ConnIndicator> mLED;

        //----------//

        private boolean mStateLED = false;
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

        SlidePanel SliderPanel = findViewById(R.id.sliderPanel);

        IpLine.setFilters(new InputFilter[] {IpFilter});
        PortLine.setFilters(new InputFilter[] {PortFilter});

        mTestHandler.setLED(findViewById(R.id.connIndicator));

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





//There is a bug that causes disconnection
///---TODO: Fix that shit---///




class NetworkThread extends HandlerThread {
    public NetworkThread(MainActivity.HandlerUI tHandler) {
        super("Network");
        mHandlerUI = tHandler;
    }

    @Override
    protected void onLooperPrepared() {
        super.onLooperPrepared();

        mHandler = new Handler(getLooper()) {
            @Override
            public void handleMessage(Message tMsg) {
                super.handleMessage(tMsg);
                Message Msg = mHandler.obtainMessage();

                switch (Header.fromInt(tMsg.what)) {
                    case REQUEST_CONN:
                        mClientUDP.sendRequest();

                        break;
                    case PING:
                        if (!mConnected) {
                            mConnected = true;
                        }

                        Msg.what = Header.PING.getValue();
                        mHandlerUI.sendMessage(Msg);

                        break;
                    case DISCONNECTED:
                        mConnected = false;

                        Msg.what = Header.DISCONNECTED.getValue();
                        mHandlerUI.sendMessage(Msg);

                        break;
                    case JOYSTICK_COORDS:
                        if (mConnected) {
                            Bundle Bund = tMsg.getData();
                            float[] Coords = Bund.getFloatArray("Coords");

                            mClientUDP.sendCoords(Coords[0], Coords[1]);

                            mCoordsObtainState = false;

                            try {
                                Thread.sleep(20);
                            } catch (InterruptedException tExcept) {
                                tExcept.printStackTrace();
                            }

                            mCoordsObtainState = true;
                        }

                        break;
                }
            }
        };

        mClientUDP = new Client(32, 2000, mHandler);
    }

    void sendConnectionRequest(String tIP, int tPort) throws SocketException, UnknownHostException {
        String IpRange = "(?:0|[1-9]|[1-9][0-9]|1[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
        Pattern IpPattern = Pattern.compile("^" + IpRange + "\\." + IpRange + "\\." + IpRange + "\\." + IpRange + "$");

        Pattern PortPattern = Pattern.compile("^([1-9]|[1-9][0-9]{1,3}|[1-5][0-9]{4}|6[0-4][0-9]{3}|65[0-4][0-9]{2}|655[0-2][0-9]|6553[0-5])$");

        Matcher IpMatch = IpPattern.matcher(tIP);
        Matcher PortMatch = PortPattern.matcher(String.valueOf(tPort));

        if (!IpMatch.matches() || !PortMatch.matches()) {
            return;
        }

        if (mHandler != null) {
            Message Msg = mHandler.obtainMessage();

            if (Msg != null) {
                if (mClientUDP.isRunning()) {
                    mClientUDP.close();
                }

                mClientUDP.init(tIP, tPort);

                System.out.println("InitRequest");

                Msg.what = Header.REQUEST_CONN.getValue();
                mHandler.sendMessage(Msg);
            }
        }
    }

    void sendJoystickCoords(float tPosX, float tPosY) {
        if (!mCoordsObtainState && (tPosX != 0 || tPosY != 0)) {
            return;
        }

        if (mHandler != null) {
            Message Msg = mHandler.obtainMessage();

            if (Msg != null) {
                Bundle Bund = new Bundle(1);
                float[] Coords = new float[2];

                Coords[0] = tPosX;
                Coords[1] = tPosY;

                Bund.putFloatArray("Coords", Coords);
                Msg.setData(Bund);
                Msg.what = Header.JOYSTICK_COORDS.getValue();

                mHandler.sendMessage(Msg);
            }
        }
    }

    //----------//

    private Handler                             mHandler                = null;
    private final MainActivity.HandlerUI        mHandlerUI;
    private Client                              mClientUDP              = null;

    private boolean                             mConnected              = false;
    private boolean                             mCoordsObtainState      = false;
}