using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//using System.Runtime.Serialization.Formatters.Binary;
#if WINDOWS_UWP
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif

public class TCPClient : MonoBehaviour
{
    #region Unity Functions

    private void Awake()
    {
        ConnectionStatusLED.material.color = Color.red;
    }
    private void OnApplicationFocus(bool focus)
    {
        if (!focus)
        {
#if WINDOWS_UWP
            StopConnection();
#endif
        }
    }
    #endregion // Unity Functions

    [SerializeField]
    List<string> hostIPAddresses = new List<string>();
    
    [SerializeField]
    string port;

    public Renderer ConnectionStatusLED;
    public bool Connected { get; private set; } = false;

    public ResearchModeVideoStream videoStream;

    public int PendingMessageCount { get; private set; } = 0;
    public int MaxPendingMessageCount = 20;

#if WINDOWS_UWP
    StreamSocket socket = null;
    public DataWriter dw;
    public DataReader dr;
    private async void StartConnection()
    {
        if(socket != null) socket.Dispose();
        socket = new StreamSocket();

        foreach(var hostIPAddress in hostIPAddresses)
        {
            try
            {
                var hostName = new Windows.Networking.HostName(hostIPAddress);
                await socket.ConnectAsync(hostName, port);
                dw = new DataWriter(socket.OutputStream);
                dr = new DataReader(socket.InputStream);
                dr.InputStreamOptions = InputStreamOptions.Partial;
                Connected = true;
                ConnectionStatusLED.material.color = Color.green;
                break;
            }
            catch (Exception ex)
            {
                SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
                Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
            }
        }
    }

    private void StopConnection()
    {
        if (videoStream.AHATRecording) videoStream.ToggleAHATRecordingEvent();

        dw?.DetachStream();
        dw?.Dispose();
        dw = null;

        dr?.DetachStream();
        dr?.Dispose();
        dr = null;

        socket?.Dispose();
        Connected = false;
        ConnectionStatusLED.material.color = Color.red;

        PendingMessageCount = 0;
    }

    public async void SendUINT16Async(string header, ushort[] data)
    {
        if (PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteString(header);

            // Write length in bytes
            dw.WriteInt32(data.Length * sizeof(ushort));

            // Write actual data
            dw.WriteBytes(UINT16ToBytes(data));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }

    public async void SendFloatAsync(string header, float[] data)
    {
        if (PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteString(header);

            // Write length in bytes
            dw.WriteInt32(data.Length * sizeof(float));
            
            // Write actual data
            dw.WriteBytes(FloatToBytes(data));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }

    // TODO: [Zikai] the convention is inconsistent with above, but we are not using them for now
    public async void SendSpatialImageAsync(byte[] LFImage, byte[] RFImage, long ts_left, long ts_right)
    {
        if (PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteString("f"); // header "f"

            // Write Length
            dw.WriteInt32(LFImage.Length + RFImage.Length);
            dw.WriteInt64(ts_left);
            dw.WriteInt64(ts_right);

            // Write actual data
            dw.WriteBytes(LFImage);
            dw.WriteBytes(RFImage);

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }


    public async void SendSpatialImageAsync(byte[] LRFImage, long ts_left, long ts_right)
    {
        if (PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteString("f"); // header "f"

            // Write Timestamp and Length
            dw.WriteInt32(LRFImage.Length);
            dw.WriteInt64(ts_left);
            dw.WriteInt64(ts_right);

            // Write actual data
            dw.WriteBytes(LRFImage);

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }

#endif


    #region Helper Function
    byte[] UINT16ToBytes(ushort[] data)
    {
        byte[] ushortInBytes = new byte[data.Length * sizeof(ushort)];
        System.Buffer.BlockCopy(data, 0, ushortInBytes, 0, ushortInBytes.Length);
        return ushortInBytes;
    }
    byte[] FloatToBytes(float[] data)
    {
        byte[] floatInBytes = new byte[data.Length * sizeof(float)];
        System.Buffer.BlockCopy(data, 0, floatInBytes, 0, floatInBytes.Length);
        return floatInBytes;
    }
    #endregion

    #region Button Callback
    public void ConnectToServerEvent()
    {
#if WINDOWS_UWP
        if (!Connected) StartConnection();
        else StopConnection();
#endif
    }
    #endregion
}
