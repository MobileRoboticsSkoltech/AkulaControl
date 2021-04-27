package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.Message;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;

import java.lang.ref.WeakReference;
import java.net.SocketException;
import java.net.UnknownHostException;

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

        Joystick Joy = findViewById(R.id.joystick);
        EditText IpLine = findViewById(R.id.ipText);
        EditText PortLine = findViewById(R.id.portText);
        Button RequestButton = findViewById(R.id.requestButton);

        SlidePanel SliderPanel = findViewById(R.id.sliderPanel);

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
                Message Msg = new Message();

                switch (Header.fromInt(tMsg.what)) {
                    case REQUEST_CONN:
                        mClientUDP.sendRequest();

                        break;
                    case PING:
                        Msg.what = Header.PING.getValue();
                        mHandlerUI.sendMessage(Msg);

                        break;
                    case DISCONNECTED:
                        Msg.what = Header.DISCONNECTED.getValue();
                        mHandlerUI.sendMessage(Msg);

                        break;
                }
            }
        };

        mClientUDP = new Client(32, 1000, mHandler);
    }

    void sendConnectionRequest(String tIP, int tPort) throws SocketException, UnknownHostException {
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

    //----------//

    private Handler                             mHandler                = null;
    private final MainActivity.HandlerUI        mHandlerUI;
    private Client                              mClientUDP              = null;

    private String                              mIP;
    private int                                 mPort;
}