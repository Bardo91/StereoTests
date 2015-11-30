package com.aeroarms.grvc.remoteviewer;

import android.graphics.Bitmap;

import java.io.IOException;
import java.net.InetAddress;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.util.concurrent.Semaphore;

/**
 * Created by Bardo91 on 13/07/2015.
 */
public class ImageReceiver {

    public int mWidth = 0;
    public int mHeigth = 0;
    public int mChannels= 0;
    // Public Interface
    public ImageReceiver(String _host, int _port){
        mHost = _host;
        mPort = _port;
        mMutex = new Semaphore(1, true);
        connect();
    }

    public boolean isConnected(){
        return mIsConnected;
    }

    public Bitmap lastFrame(){
        if(isConnected()) {
            Bitmap frame = null;
            try {
                mMutex.acquire();
                frame = mLastFrame;
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                mMutex.release();
            }
            return frame;
        }else{
            return null;
        }
    }

    public void stop(){
        mIsConnected = false;
        try {
            Thread.sleep(1000); // sleep 1 second
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        connect();
    }

    // Private Interface
    private void connect(){
        Thread connectThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(!isConnected()){
                    try {
                        InetAddress serverAddr = InetAddress.getByName(mHost);
                        mReceiveSocket = new Socket(serverAddr, mPort);
                        Thread.sleep(1000); // sleep 1 second
                        mIsConnected = true;
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                mAcquireThread = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        acquireCallback();
                    }
                });
                mAcquireThread.start();
            }
        });
        connectThread.start();
    }

    private void acquireCallback(){
        while(isConnected()){
            if(mReceiveSocket.isConnected()){
                byte[] frameBuffer;
                int height, width, channels;
                int inLen = 0;
                try {
                    mReceiveSocket.getOutputStream().write("send".getBytes());

                    byte[] imgSizeBuf = new byte[3*4];
                    int len = mReceiveSocket.getInputStream().read(imgSizeBuf);

                    IntBuffer intBuf =  ByteBuffer.wrap(imgSizeBuf) .order(ByteOrder.LITTLE_ENDIAN).asIntBuffer();
                    int[] sizes = new int[intBuf.remaining()];
                    intBuf.get(sizes);

                    height = sizes[0];
                    width = sizes[1];
                    channels = sizes[2];

                    mWidth = width;
                    mHeigth = height;
                    mChannels = channels;

                    frameBuffer  = new byte[height*width*channels];

                    byte[] buffer = new byte[height*width*channels];
                    do {
                        len = mReceiveSocket.getInputStream().read(buffer);
                        System.arraycopy(buffer, 0, frameBuffer, inLen, len);
                        inLen += len;
                    } while(len != -1 && inLen < frameBuffer.length) ;

                } catch (IOException e) {
                    e.printStackTrace();
                    break;
                } catch (RuntimeException e){
                    e.printStackTrace();
                    break;
                }

                byte [] bits = new byte[frameBuffer.length*4/3]; //That's where the RGBA array goes.
                for(int i=0;i<frameBuffer.length/3;i++){
                    bits[i * 4] = frameBuffer[i*3 + 2]; // Convert from BGR to RGB
                    bits[i * 4 + 1] = frameBuffer[i*3 + 1];
                    bits[i * 4 + 2] = frameBuffer[i*3];
                    bits[i * 4 + 3] = -1;
                }

                Bitmap frame = Bitmap.createBitmap(width,height, Bitmap.Config.ARGB_8888);
                frame.copyPixelsFromBuffer(ByteBuffer.wrap(bits));
                try {
                    mMutex.acquire();
                    mLastFrame = frame;
                } catch (InterruptedException e) {
                    e.printStackTrace();
                } finally {
                    mMutex.release();
                }
            } else{
                mIsConnected = false;
            }
        }
    }

    private String      mHost;
    private int         mPort;
    private boolean     mIsConnected;
    private Bitmap      mLastFrame;
    private Socket      mReceiveSocket;
    private Thread      mAcquireThread;
    private Semaphore   mMutex;

}
