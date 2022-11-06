﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;

#if ENABLE_WINMD_SUPPORT
using HL2UnityPlugin;
#endif

public class ResearchModeVideoStream : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    HL2ResearchMode researchMode;
    HL2Interactions interactions;
#endif

    enum DepthSensorMode
    {
        ShortThrow,
        LongThrow,
        None
    };
    [SerializeField] DepthSensorMode depthSensorMode = DepthSensorMode.ShortThrow;
    [SerializeField] bool enablePointCloud = true;

    TCPClient tcpClient;

    public GameObject depthPreviewPlane = null;
    private Material depthMediaMaterial = null;
    private Texture2D depthMediaTexture = null;
    private byte[] depthFrameData = null;

    public GameObject shortAbImagePreviewPlane = null;
    private Material shortAbImageMediaMaterial = null;
    private Texture2D shortAbImageMediaTexture = null;
    private byte[] shortAbImageFrameData = null;

    public GameObject longDepthPreviewPlane = null;
    private Material longDepthMediaMaterial = null;
    private Texture2D longDepthMediaTexture = null;
    private byte[] longDepthFrameData = null;

    public GameObject longAbImagePreviewPlane = null;
    private Material longAbImageMediaMaterial = null;
    private Texture2D longAbImageMediaTexture = null;
    private byte[] longAbImageFrameData = null;

    public GameObject LFPreviewPlane = null;
    private Material LFMediaMaterial = null;
    private Texture2D LFMediaTexture = null;
    private byte[] LFFrameData = null;

    public GameObject RFPreviewPlane = null;
    private Material RFMediaMaterial = null;
    private Texture2D RFMediaTexture = null;
    private byte[] RFFrameData = null;

    public UnityEngine.UI.Text text;

    public GameObject pointCloudRendererGo;
    public Color pointColor = Color.white;
    private PointCloudRenderer pointCloudRenderer;
#if ENABLE_WINMD_SUPPORT
    Windows.Perception.Spatial.SpatialCoordinateSystem unityWorldOrigin;
#endif

    public Renderer ahatRecordingLED;
    private bool ahatRecording = false;
    public bool AHATRecording {
        get { return ahatRecording; }
    }
    private bool ahatLUTSent = false;

    private void Awake()
    {
        ahatRecordingLED.material.color = Color.red;
#if ENABLE_WINMD_SUPPORT
#if UNITY_2020_1_OR_NEWER // note: Unity 2021.2 and later not supported
        IntPtr WorldOriginPtr = UnityEngine.XR.WindowsMR.WindowsMREnvironment.OriginSpatialCoordinateSystem;
        unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;
        //unityWorldOrigin = Windows.Perception.Spatial.SpatialLocator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem;
#else
        IntPtr WorldOriginPtr = UnityEngine.XR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr();
        unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;
#endif
#endif
    }
    void Start()
    {
        if (depthSensorMode == DepthSensorMode.ShortThrow)
        {
            if (depthPreviewPlane != null)
            {
                depthMediaMaterial = depthPreviewPlane.GetComponent<MeshRenderer>().material;
                depthMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
                depthMediaMaterial.mainTexture = depthMediaTexture;
            }

            if (shortAbImagePreviewPlane != null)
            {
                shortAbImageMediaMaterial = shortAbImagePreviewPlane.GetComponent<MeshRenderer>().material;
                shortAbImageMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
                shortAbImageMediaMaterial.mainTexture = shortAbImageMediaTexture;
            }
            longDepthPreviewPlane.SetActive(false);
            longAbImagePreviewPlane.SetActive(false);
        }
        
        if (depthSensorMode == DepthSensorMode.LongThrow)
        {
            if (longDepthPreviewPlane != null)
            {
                longDepthMediaMaterial = longDepthPreviewPlane.GetComponent<MeshRenderer>().material;
                longDepthMediaTexture = new Texture2D(320, 288, TextureFormat.Alpha8, false);
                longDepthMediaMaterial.mainTexture = longDepthMediaTexture;
            }

            if (longAbImagePreviewPlane != null)
            {
                longAbImageMediaMaterial = longAbImagePreviewPlane.GetComponent<MeshRenderer>().material;
                longAbImageMediaTexture = new Texture2D(320, 288, TextureFormat.Alpha8, false);
                longAbImageMediaMaterial.mainTexture = longAbImageMediaTexture;
            }
            depthPreviewPlane.SetActive(false);
            shortAbImagePreviewPlane.SetActive(false);
        }
        

        if (LFPreviewPlane != null)
        {
            LFMediaMaterial = LFPreviewPlane.GetComponent<MeshRenderer>().material;
            LFMediaTexture = new Texture2D(640, 480, TextureFormat.Alpha8, false);
            LFMediaMaterial.mainTexture = LFMediaTexture;
        }

        if (RFPreviewPlane != null)
        {
            RFMediaMaterial = RFPreviewPlane.GetComponent<MeshRenderer>().material;
            RFMediaTexture = new Texture2D(640, 480, TextureFormat.Alpha8, false);
            RFMediaMaterial.mainTexture = RFMediaTexture;
        }

        if (pointCloudRendererGo != null)
        {
            pointCloudRenderer = pointCloudRendererGo.GetComponent<PointCloudRenderer>();
        }

        tcpClient = GetComponent<TCPClient>();

#if ENABLE_WINMD_SUPPORT
        researchMode = new HL2ResearchMode();

        // Depth sensor should be initialized in only one mode
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.InitializeLongDepthSensor();
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.InitializeDepthSensor();
        
        researchMode.InitializeSpatialCamerasFront();
        researchMode.SetReferenceCoordinateSystem(unityWorldOrigin);
        researchMode.SetPointCloudDepthOffset(0);

        // Depth sensor should be initialized in only one mode
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.StartLongDepthSensorLoop(enablePointCloud);
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.StartDepthSensorLoop(false, false, enablePointCloud);

        researchMode.StartSpatialCamerasFrontLoop();

        UnityEngine.WSA.Application.InvokeOnUIThread(() =>
        {
            interactions = new HL2Interactions();  // must be invoked on the UI thread since it needs SpatialInteractionManager
        }, true);
        interactions.SetReferenceCoordinateSystem(unityWorldOrigin);
        interactions.EnableEyeTracking();
#endif
    }

    bool startRealtimePreview = false;
    void LateUpdate()
    {
#if ENABLE_WINMD_SUPPORT
        bool ahatUpdated = false;

        if (depthSensorMode == DepthSensorMode.ShortThrow && researchMode.DepthMapTextureUpdated()) 
        {
            /*
             * DepthMapTextureUpdated() and ShortAbImageTextureUpdated() are set to true at the same time (HL2UnityPlugin/HL2ResearchMode.cpp:342)
             * As m_depthMapTextureUpdated is set later then m_shortAbImageTextureUpdated, we use it as the flag to avoid aync problem.
             */
            ahatUpdated = true;
        }

        // update depth map texture
        if (depthSensorMode == DepthSensorMode.ShortThrow && startRealtimePreview && 
            depthPreviewPlane != null && researchMode.DepthMapTextureUpdated())
        {
            
            byte[] frameTexture = researchMode.GetDepthMapTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (depthFrameData == null)
                {
                    depthFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, depthFrameData, 0, depthFrameData.Length);
                }

                depthMediaTexture.LoadRawTextureData(depthFrameData);
                depthMediaTexture.Apply();
            }
        }

        // update short-throw AbImage texture
        if (depthSensorMode == DepthSensorMode.ShortThrow && startRealtimePreview && 
            shortAbImagePreviewPlane != null && researchMode.ShortAbImageTextureUpdated())
        {
            // not updating ahatUpdated, see above
            byte[] frameTexture = researchMode.GetShortAbImageTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (shortAbImageFrameData == null)
                {
                    shortAbImageFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, shortAbImageFrameData, 0, shortAbImageFrameData.Length);
                }

                shortAbImageMediaTexture.LoadRawTextureData(shortAbImageFrameData);
                shortAbImageMediaTexture.Apply();
            }
        }

        if (ahatRecording && ahatUpdated) 
        {
            if (!ahatLUTSent) 
            {
                if (SendAHATLUTData())
                {
                    ahatLUTSent = true;
                    ahatRecordingLED.material.color = Color.green;
                }
            }

            if (ahatLUTSent)
            {
                // Measurements shows that the following call only takes 1-2ms
                interactions.Update(researchMode.GetDepthUpdateTimestamp());
                SendAHATDepthData();
                SendInteractionData();
            }
        }

        // update long depth map texture
        if (depthSensorMode == DepthSensorMode.LongThrow && startRealtimePreview && 
            longDepthPreviewPlane != null && researchMode.LongDepthMapTextureUpdated())
        {
            byte[] frameTexture = researchMode.GetLongDepthMapTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (longDepthFrameData == null)
                {
                    longDepthFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, longDepthFrameData, 0, longDepthFrameData.Length);
                }

                longDepthMediaTexture.LoadRawTextureData(longDepthFrameData);
                longDepthMediaTexture.Apply();
            }
        }

        // update long-throw AbImage texture
        if (depthSensorMode == DepthSensorMode.LongThrow && startRealtimePreview &&
            longAbImagePreviewPlane != null && researchMode.LongAbImageTextureUpdated())
        {
            byte[] frameTexture = researchMode.GetLongAbImageTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (longAbImageFrameData == null)
                {
                    longAbImageFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, longAbImageFrameData, 0, longAbImageFrameData.Length);
                }

                longAbImageMediaTexture.LoadRawTextureData(longAbImageFrameData);
                longAbImageMediaTexture.Apply();
            }
        }

        // update LF camera texture
        if (startRealtimePreview && LFPreviewPlane != null && researchMode.LFImageUpdated())
        {
            long ts;
            byte[] frameTexture = researchMode.GetLFCameraBuffer(out ts);
            if (frameTexture.Length > 0)
            {
                if (LFFrameData == null)
                {
                    LFFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, LFFrameData, 0, LFFrameData.Length);
                }

                LFMediaTexture.LoadRawTextureData(LFFrameData);
                LFMediaTexture.Apply();
            }
        }
        // update RF camera texture
        if (startRealtimePreview && RFPreviewPlane != null && researchMode.RFImageUpdated())
        {
            long ts;
            byte[] frameTexture = researchMode.GetRFCameraBuffer(out ts);
            if (frameTexture.Length > 0)
            {
                if (RFFrameData == null)
                {
                    RFFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, RFFrameData, 0, RFFrameData.Length);
                }

                RFMediaTexture.LoadRawTextureData(RFFrameData);
                RFMediaTexture.Apply();
            }
        }

        // Update point cloud
        UpdatePointCloud();
#endif
    }

#if ENABLE_WINMD_SUPPORT
    private void UpdatePointCloud()
    {
        if (enablePointCloud && renderPointCloud && pointCloudRendererGo != null)
        {
            if ((depthSensorMode == DepthSensorMode.LongThrow && !researchMode.LongThrowPointCloudUpdated()) ||
                (depthSensorMode == DepthSensorMode.ShortThrow && !researchMode.PointCloudUpdated())) return;

            float[] pointCloud = new float[] { };
            if (depthSensorMode == DepthSensorMode.LongThrow) pointCloud = researchMode.GetLongThrowPointCloudBuffer();
            else if (depthSensorMode == DepthSensorMode.ShortThrow) pointCloud = researchMode.GetPointCloudBuffer();
            
            if (pointCloud.Length > 0)
            {
                int pointCloudLength = pointCloud.Length / 3;
                Vector3[] pointCloudVector3 = new Vector3[pointCloudLength];
                for (int i = 0; i < pointCloudLength; i++)
                {
                    pointCloudVector3[i] = new Vector3(pointCloud[3 * i], pointCloud[3 * i + 1], pointCloud[3 * i + 2]);
                }
                if (depthSensorMode == DepthSensorMode.ShortThrow) 
                {
                    text.text = "AHAT ";
                } else if (depthSensorMode == DepthSensorMode.LongThrow) 
                {
                    text.text = "Long-Throw ";
                }
                text.text += "Point Cloud: " + pointCloudVector3.Length.ToString();
                if (tcpClient != null)
                {
                    text.text += "\n"+ "TCP Pending: " + tcpClient.PendingMessageCount.ToString();
                }
                pointCloudRenderer.Render(pointCloudVector3, pointColor);
            }
        }
    }
#endif


    #region Button Event Functions
    public void TogglePreviewEvent()
    {
        startRealtimePreview = !startRealtimePreview;
    }

    bool renderPointCloud = true;
    public void TogglePointCloudEvent()
    {
        renderPointCloud = !renderPointCloud;
        if (renderPointCloud)
        {
            pointCloudRendererGo.SetActive(true);
        }
        else
        {
            pointCloudRendererGo.SetActive(false);
        }
    }

    public void StopSensorsEvent()
    {
#if ENABLE_WINMD_SUPPORT
        researchMode.StopAllSensorDevice();
#endif
        startRealtimePreview = false;
    }

    public void ToggleAHATRecordingEvent() {
        if (!ahatRecording) {
            if (tcpClient.Connected) {
                ahatRecording = true;
                ahatLUTSent = false;
                ahatRecordingLED.material.color = Color.yellow;

#if ENABLE_WINMD_SUPPORT
                var data = new List<float>();
                SerializeMatrix4x4Transposed(data, researchMode.GetDepthExtrinsics());
                tcpClient.SendFloatAsync("e", data.ToArray());
#endif
            }
            
        } else {
            ahatRecording = false;
            ahatRecordingLED.material.color = Color.red;
        }
    }

    public void SaveSpatialImageEvent()
    {
#if ENABLE_WINMD_SUPPORT
#if WINDOWS_UWP
        long ts_ft_left, ts_ft_right;
        var LRFImage = researchMode.GetLRFCameraBuffer(out ts_ft_left, out ts_ft_right);
        Windows.Perception.PerceptionTimestamp ts_left = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(DateTime.FromFileTime(ts_ft_left));
        Windows.Perception.PerceptionTimestamp ts_right = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(DateTime.FromFileTime(ts_ft_right));

        long ts_unix_left = ts_left.TargetTime.ToUnixTimeMilliseconds();
        long ts_unix_right = ts_right.TargetTime.ToUnixTimeMilliseconds();
        long ts_unix_current = GetCurrentTimestampUnix();

        text.text = "Left: " + ts_unix_left.ToString() + "\n" +
            "Right: " + ts_unix_right.ToString() + "\n" +
            "Current: " + ts_unix_current.ToString();

        if (tcpClient != null)
        {
            tcpClient.SendSpatialImageAsync(LRFImage, ts_unix_left, ts_unix_right);
        }
#endif
#endif
    }

    #endregion

    public bool SendAHATLUTData()
    {
#if ENABLE_WINMD_SUPPORT
        var ahatLUT = researchMode.GetDepthLUT();
        if (ahatLUT.Length == 0) return false;
#if WINDOWS_UWP
        if (tcpClient != null)
        {
            tcpClient.SendFloatAsync("l", ahatLUT);
            return true;  // here we just assume the sent will succeed
        }
#endif
#endif
        return false;
    }

    public void SendAHATDepthData()
    {
#if ENABLE_WINMD_SUPPORT
        var depthMap = researchMode.GetDepthMapBuffer();
        var rigToWorld = researchMode.GetRigToWorld();
#if WINDOWS_UWP
        if (tcpClient != null)
        {
            tcpClient.SendUINT16Async("d", depthMap);

            var data = new List<float>();
            SerializeMatrix4x4Transposed(data, rigToWorld);
            tcpClient.SendFloatAsync("r", data.ToArray());
        }
#endif
#endif
    }

    public void SendInteractionData()
    {
#if ENABLE_WINMD_SUPPORT
        List<float> data = new List<float>();

        // Head
        SerializeMatrix4x4Transposed(data, interactions.GetHeadTransform());

        // Hands
        for (int i = 0; i < (int) HandIndex.Count; i++)
        {
            var handIndex = (HandIndex) i;
            data.Add(interactions.IsHandTracked(handIndex) ? 1.0f : 0.0f);
            for (int j = 0; j < (int) HandJointIndex.Count; j++)
            {
                // Different with StreamRecorder: not-tracked hand is not zeroed
                SerializeMatrix4x4Transposed(data, interactions.GetOrientedJoint(handIndex, (HandJointIndex) j));
            }
        }

        // Eye
        data.Add(interactions.IsEyeTrackingActive() ? 1.0f : 0.0f);
        SerializeVector4(data, interactions.GetEyeGazeOrigin());
        SerializeVector4(data, interactions.GetEyeGazeDirection());

#if WINDOWS_UWP
        if (tcpClient != null)
        {
            tcpClient.SendFloatAsync("i", data.ToArray());
        }
#endif
#endif
    }

    static private void SerializeMatrix4x4Transposed(List<float> l, System.Numerics.Matrix4x4 matrix) 
    {
        // In column major order
        l.Add(matrix.M11);
        l.Add(matrix.M21);
        l.Add(matrix.M31);
        l.Add(matrix.M41);
        l.Add(matrix.M12);
        l.Add(matrix.M22);
        l.Add(matrix.M32);
        l.Add(matrix.M42);
        l.Add(matrix.M13);
        l.Add(matrix.M23);
        l.Add(matrix.M33);
        l.Add(matrix.M43);
        l.Add(matrix.M14);
        l.Add(matrix.M24);
        l.Add(matrix.M34);
        l.Add(matrix.M44);
    }

    static private void SerializeVector4(List<float> l, System.Numerics.Vector4 vector) 
    {
        l.Add(vector.X);
        l.Add(vector.Y);
        l.Add(vector.Z);
        l.Add(vector.W);
    }

    private void OnApplicationFocus(bool focus)
    {
        if (!focus) StopSensorsEvent();
    }

#if WINDOWS_UWP
    private long GetCurrentTimestampUnix()
    {
        // Get the current time, in order to create a PerceptionTimestamp. 
        Windows.Globalization.Calendar c = new Windows.Globalization.Calendar();
        Windows.Perception.PerceptionTimestamp ts = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime());
        return ts.TargetTime.ToUnixTimeMilliseconds();
        //return ts.SystemRelativeTargetTime.Ticks;
    }
    private Windows.Perception.PerceptionTimestamp GetCurrentTimestamp()
    {
        // Get the current time, in order to create a PerceptionTimestamp. 
        Windows.Globalization.Calendar c = new Windows.Globalization.Calendar();
        return Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime());
    }
#endif
}