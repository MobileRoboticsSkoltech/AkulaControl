package com.example.androidapp;
//-----------------------------//
import android.os.Build;
import android.os.Message;

import androidx.annotation.RequiresApi;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;
import java.time.Duration;
import java.time.LocalTime;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
//-----------------------------//
public class Client {
    public Client(int tPacketSize, int tConnTimeout, android.os.Handler tHandler) {
        mPacketSize     = tPacketSize;
        mTimeoutMs      = tConnTimeout;
        mHandler        = tHandler;

        mReadBuffer     = new byte[tPacketSize];
        mWriteBuffer    = new byte[tPacketSize];

        mTimerThread.start();
    }

    //----------//

    public void init(String tAddress, int tPort) throws SocketException, UnknownHostException {
        mAddress            = InetAddress.getByName(tAddress);
        mPort               = tPort;

        mSocket             = new DatagramSocket();

        mSocket.setSoTimeout(1000);

        mReadThread         = new Thread(this::readFunc);
        mWriteThread        = new Thread(this::writeFunc);
        mProcessThread      = new Thread(this::processFunc);

        mReadThread.start();
        mWriteThread.start();
        mProcessThread.start();

        mRunning = true;
    }

    public void close() {
        mTerminate.set(true);
        signalAll();

        try {
            mReadThread.join();
            mWriteThread.join();
            mProcessThread.join();
        } catch (InterruptedException ignored) {
        }

        mSocket.close();

        mTerminate.set(false);

        mRunning = false;
    }

    public void shutDown() throws InterruptedException {
        mTerminate.set(true);
        mShutDown.set(true);
        signalAll();

        mTimerThread.join();
    }

    public void sendRequest() {
        byte[] Packet = new byte[mPacketSize];

        Header Tag = Header.REQUEST_CONN;
        byte[] Zeros = new byte[mPacketSize - 4];

        ByteBuffer Buffer = ByteBuffer.wrap(Packet);

        Buffer.putInt(Tag.getValue());
        Buffer.put(Zeros);

        fillWriteBuffer(Packet);
    }

    public boolean isRunning() {
        return mRunning;
    }

    //----------//

    void readFunc() {
        byte[]              Packet      = new byte[mPacketSize];
        DatagramPacket      PacketUDP   = new DatagramPacket(Packet, Packet.length);
        boolean             NewData     = false;    //---Do I really need this?---//

        while (!NewData && !mTerminate.get()) {
            try {
                mSocket.receive(PacketUDP);
                NewData = true;
            } catch (SocketTimeoutException tExcept) {
                System.err.println("readFunc: receive timeout!");
                continue;
            } catch (IOException tExcept) {
                System.err.println("readFunc: receive exception!");
                tExcept.printStackTrace();

                mShutDown.set(true);
                signalAll();

                return;
            }

            if (!fillReadBuffer(Packet)) {
                return;
            }

            NewData = false;
        }
    }
    @RequiresApi(api = Build.VERSION_CODES.O)
    void writeFunc() {
        byte[] Packet = new byte[mPacketSize];

        while (!mTerminate.get()) {
            synchronized (mWriteMonitor) {
                while (!mWriteState && !mTerminate.get()) {
                    try {
                        mWriteMonitor.wait();
                    } catch (InterruptedException tExcept)  {
                        Thread.currentThread().interrupt();
                        System.err.println("writeFunc: thread interrupted!");

                        mShutDown.set(true);
                        signalAll();

                        return;
                    }
                }

                if (mTerminate.get()) {
                    return;
                }

                if (mConnected.get()) {
                    synchronized (mTimerMonitor) {
                        mLastPacketTime = LocalTime.now();
                    }
                }

                if (!getWriteBuffer(Packet)) {
                    return;
                }

                DatagramPacket PacketUDP = new DatagramPacket(Packet, Packet.length, mAddress, mPort);

                try {
                    mSocket.send(PacketUDP);
                } catch (IOException tExcept) {
                    System.err.println("writeFunc: send exception!");

                    mShutDown.set(true);
                    signalAll();

                    return;
                }

                mWriteState = false;
            }
        }
    }
    @RequiresApi(api = Build.VERSION_CODES.O)
    void processFunc() {
        byte[] Packet = new byte[mPacketSize];
        Header Tag;

        while (!mTerminate.get()) {
            synchronized (mProcessMonitor) {
                while (!mProcessState && !mTerminate.get()) {
                    try {
                        mProcessMonitor.wait();
                    } catch (InterruptedException tExcept)  {
                        Thread.currentThread().interrupt();
                        System.err.println("writeFunc: thread interrupted!");

                        mShutDown.set(true);
                        signalAll();

                        return;
                    }
                }

                if (mTerminate.get()) {
                    return;
                }

                if (!getReadBuffer(Packet)) {
                    return;
                }

                ByteBuffer Buffer = ByteBuffer.wrap(Packet);

                Tag = Header.fromInt(Integer.reverseBytes(Buffer.getInt()));

                switch (Objects.requireNonNull(Tag)) {
                    case REQUEST_CONN:
                    case JOYSTICK_COORDS:
                        break;
                    case PING:
                        if (!mConnected.get()) {
                            mConnected.set(true);

                            synchronized (mTimerMonitor) {
                                mLastPacketTime = LocalTime.now();
                            }
                        }

                        byte[] PingPacket = new byte[mPacketSize];

                        Header PingTag = Header.PING;
                        byte[] Zeros = new byte[mPacketSize - 4];

                        ByteBuffer PingBuffer = ByteBuffer.wrap(PingPacket);

                        PingBuffer.putInt(Integer.reverseBytes(PingTag.getValue()));
                        PingBuffer.put(Zeros);

                        if (!fillWriteBuffer(PingPacket)) {
                            return;
                        }

                        Message Msg = new Message();
                        Msg.what = Header.PING.getValue();

                        mHandler.sendMessage(Msg);

                        break;
                    case STATUS:
                        break;
                    case INVALID:
                }

                mProcessState = false;
            }
        }
    }

    //----------//

    boolean fillReadBuffer(byte[] tBuffer) {
        synchronized (mReadBufferMonitor) {
            while (mReadBufferState && !mTerminate.get()) {
                try {
                    mReadBufferMonitor.wait();
                } catch (InterruptedException tExcept)  {
                    Thread.currentThread().interrupt();
                    System.err.println("fillReadBuffer: thread interrupted!");

                    mShutDown.set(true);
                    signalAll();

                    return false;
                }
            }

            if (mTerminate.get()) {
                return false;
            }

            System.arraycopy(tBuffer, 0, mReadBuffer, 0, mPacketSize);

            synchronized (mProcessMonitor) {
                mProcessState = true;
                mProcessMonitor.notify();
            }

            mReadBufferState = true;
            mReadBufferMonitor.notifyAll();
        }

        return true;
    }
    boolean getReadBuffer(byte[] tBuffer) {
        synchronized (mReadBufferMonitor) {
            while (!mReadBufferState && !mTerminate.get()) {
                try {
                    mReadBufferMonitor.wait();
                } catch (InterruptedException tExcept)  {
                    Thread.currentThread().interrupt();
                    System.err.println("fillReadBuffer: thread interrupted!");

                    mShutDown.set(true);
                    signalAll();

                    return false;
                }
            }

            if (mTerminate.get()) {
                return false;
            }

            System.arraycopy(mReadBuffer, 0, tBuffer, 0, mPacketSize);

            mReadBufferState = false;
            mReadBufferMonitor.notifyAll();
        }

        return true;
    }

    boolean fillWriteBuffer(byte[] tBuffer) {
        synchronized (mWriteBufferMonitor) {
            while (mWriteBufferState && !mTerminate.get()) {
                try {
                    mWriteBufferMonitor.wait();
                } catch (InterruptedException tExcept)  {
                    Thread.currentThread().interrupt();
                    System.err.println("fillWriteBuffer: thread interrupted!");

                    mShutDown.set(true);
                    signalAll();

                    return false;
                }
            }

            if (mTerminate.get()) {
                return false;
            }

            System.arraycopy(tBuffer, 0, mWriteBuffer, 0, mPacketSize);

            synchronized (mWriteMonitor) {
                mWriteState = true;
                mWriteMonitor.notify();
            }

            mWriteBufferState = true;
            mWriteBufferMonitor.notifyAll();
        }

        return true;
    }
    boolean getWriteBuffer(byte[] tBuffer) {
        synchronized (mWriteBufferMonitor) {
            while (!mWriteBufferState && !mTerminate.get()) {
                try {
                    mWriteBufferMonitor.wait();
                } catch (InterruptedException tExcept)  {
                    Thread.currentThread().interrupt();
                    System.err.println("fillWriteBuffer: thread interrupted!");

                    mShutDown.set(true);
                    signalAll();

                    return false;
                }
            }

            if (mTerminate.get()) {
                return false;
            }

            System.arraycopy(mWriteBuffer, 0, tBuffer, 0, mPacketSize);

            mWriteBufferState = false;
            mWriteBufferMonitor.notifyAll();
        }

        return true;
    }

    //----------//

    @RequiresApi(api = Build.VERSION_CODES.O)
    void timerFunc() {
        while (true) {
            synchronized (mTimerMonitor) {
                if (mConnected.get() && Duration.between(mLastPacketTime, LocalTime.now()).toMillis() > mTimeoutMs) {
                    mConnected.set(false);
                    close();

                    Message Msg = new Message();
                    Msg.what = Header.DISCONNECTED.getValue();

                    mHandler.sendMessage(Msg);
                }
            }

            try {
                Thread.sleep(500);
            } catch (InterruptedException tExcept) {
                mShutDown.set(true);
                signalAll();
            }

            if (mShutDown.get()) {
                try {
                    mReadThread.join();
                    mWriteThread.join();
                    mProcessThread.join();
                } catch (InterruptedException tExcept) {
                    return;
                }

                mSocket.close();

                return;
            }
        }
    }

    //----------//

    void signalAll() {
        synchronized (mWriteMonitor) {
            mWriteMonitor.notifyAll();
        }

        synchronized (mProcessMonitor) {
            mProcessMonitor.notifyAll();
        }

        synchronized (mReadBufferMonitor) {
            mReadBufferMonitor.notifyAll();
        }

        synchronized (mWriteBufferMonitor) {
            mWriteBufferMonitor.notifyAll();
        }
    }

    //----------//

    private final android.os.Handler    mHandler;

    //----------//

    private final AtomicBoolean         mTerminate                  = new AtomicBoolean(false);
    private final AtomicBoolean         mConnected                  = new AtomicBoolean(false);
    private final AtomicBoolean         mShutDown                   = new AtomicBoolean(false);

    private boolean                     mRunning                    = false;

    LocalTime                           mLastPacketTime;
    int                                 mTimeoutMs;

    //----------//

    private DatagramSocket              mSocket;

    private int                         mPort;
    private InetAddress                 mAddress;

    //----------//

    private final Integer               mPacketSize;

    private final byte[]                mReadBuffer;
    private final byte[]                mWriteBuffer;

    //----------//

    private Thread                      mReadThread;
    private Thread                      mWriteThread;
    private Thread                      mProcessThread;

    private final Thread                mTimerThread                = new Thread(this::timerFunc);

    private final Object                mWriteMonitor               = new Object();
    private final Object                mProcessMonitor             = new Object();

    private boolean                     mWriteState                 = false;
    private boolean                     mProcessState               = false;

    private final Object                mTimerMonitor               = new Object();

    //----------//

    private final Object                mReadBufferMonitor          = new Object();
    private final Object                mWriteBufferMonitor         = new Object();

    private boolean                     mReadBufferState            = false;
    private boolean                     mWriteBufferState           = false;
}
