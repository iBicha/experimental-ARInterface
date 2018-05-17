using System;
using System.Collections.Generic;
using UnityEngine;
using GoogleARCore;
using System.Collections;
using System.Runtime.InteropServices;
using UnityEngine.XR;

namespace UnityARInterface
{
    public class ARCoreInterface : ARInterface
    {       
        private List<DetectedPlane> m_DetectedPlaneBuffer = new List<DetectedPlane>();
        private ScreenOrientation m_CachedScreenOrientation;
        private Dictionary<DetectedPlane, BoundedPlane> m_DetectedPlanes = new Dictionary<DetectedPlane, BoundedPlane>();
        private ARCoreSession m_ARCoreSession;
        private ARCoreSessionConfig m_ARCoreSessionConfig;
        private ARBackgroundRenderer m_BackgroundRenderer;
        private Matrix4x4 m_DisplayTransform = Matrix4x4.identity;
        private List<Vector4> m_TempPointCloud = new List<Vector4>();
        private Dictionary<ARAnchor, Anchor> m_Anchors = new Dictionary<ARAnchor, Anchor>();
        private bool m_BackgroundRendering;

        public override bool IsSupported
        {
            get
            {
                return
                    Session.Status != SessionStatus.ErrorApkNotAvailable &&
                    Session.Status != SessionStatus.ErrorSessionConfigurationNotSupported;
            }
        }

        public override bool BackgroundRendering
        {
            get
            {
                return m_BackgroundRendering;
            }
            set
            {
                if (m_BackgroundRenderer == null)
                    return;

                m_BackgroundRendering = value;
                m_BackgroundRenderer.mode = m_BackgroundRendering ? 
                    ARRenderMode.MaterialAsBackground : ARRenderMode.StandardBackground;
            }
        }

        public override IEnumerator StartService(Settings settings)
        {
            if (m_ARCoreSessionConfig == null)
                m_ARCoreSessionConfig = ScriptableObject.CreateInstance<ARCoreSessionConfig>();

            m_ARCoreSessionConfig.EnableLightEstimation = settings.enableLightEstimation;
            m_ARCoreSessionConfig.PlaneFindingMode = settings.enablePlaneDetection ? DetectedPlaneFindingMode.HorizontalAndVertical : DetectedPlaneFindingMode.Disabled;
            //Do we want to match framerate to the camera?
            m_ARCoreSessionConfig.MatchCameraFramerate = true;

            // Create a GameObject on which the session component will live.
            if (m_ARCoreSession == null)
            {
                var go = new GameObject("ARCore Session");
                go.SetActive(false);
                m_ARCoreSession = go.AddComponent<ARCoreSession>();
                m_ARCoreSession.SessionConfig = m_ARCoreSessionConfig;
                go.SetActive(true);
            }

            // Enabling the session triggers the connection
            m_ARCoreSession.SessionConfig = m_ARCoreSessionConfig;
            m_ARCoreSession.enabled = true;

            if (!IsSupported)
            {
                switch (Session.Status)
                {
                    case SessionStatus.ErrorApkNotAvailable:
                        Debug.LogError("ARCore APK is not installed");
                        yield break;
                    case SessionStatus.ErrorPermissionNotGranted:
                        Debug.LogError("A needed permission (likely the camera) has not been granted");
                        yield break;
                    case SessionStatus.ErrorSessionConfigurationNotSupported:
                        Debug.LogError("The given ARCore session configuration is not supported on this device");
                        yield break;
                    case SessionStatus.FatalError:
                        Debug.LogError("A fatal error was encountered trying to start the ARCore session");
                        yield break;
                }
            }

            while (!Session.Status.IsValid())
            {
                IsRunning = false;

                if (Session.Status.IsError())
                {
                    switch (Session.Status)
                    {
                        case SessionStatus.ErrorPermissionNotGranted:
                            Debug.LogError("A needed permission (likely the camera) has not been granted");
                            yield break;
                        case SessionStatus.FatalError:
                            Debug.LogError("A fatal error was encountered trying to start the ARCore session");
                            yield break;
                    }
                }

                yield return null;
            }

            // If we make it out of the while loop, then the session is initialized and valid
            IsRunning = true;

        }

        public override void StopService()
        {
            var anchors = m_Anchors.Keys;
            foreach (var anchor in anchors)
            {
                DestroyAnchor(anchor);
            }

            m_ARCoreSession.enabled = false;
            BackgroundRendering = false;
            m_BackgroundRenderer.backgroundMaterial = null;
            m_BackgroundRenderer.camera = null;
            m_BackgroundRenderer = null;
            IsRunning = false;
        }

        public override bool TryGetUnscaledPose(ref Pose pose)
        {
            if (Session.Status != SessionStatus.Tracking)
                return false;

            pose.position = Frame.Pose.position;
            pose.rotation = Frame.Pose.rotation;
            return true;
        }

        public override bool TryGetCameraImage(ref CameraImage cameraImage)
        {
            const bool LuminanceOnly = false;
            
            using (var imageBytes = Frame.CameraImage.AcquireCameraImageBytes())
            {
                if (!imageBytes.IsAvailable)
                    return false;

                cameraImage.width = imageBytes.Width;
                cameraImage.height = imageBytes.Height;
                
                if (imageBytes.YRowStride != imageBytes.Width)
                {
                    //Y shoud be 1 byte per pixel. Not doing anything otherwise.
                    return false;
                }

                //We expect 1 byte per pixel for Y
                int bufferSize = imageBytes.Width * imageBytes.Height;
                if(cameraImage.y == null || cameraImage.y.Length != bufferSize)
                    cameraImage.y = new byte[bufferSize];

                //Y plane is copied as is.
                Marshal.Copy(imageBytes.Y, cameraImage.y, 0, bufferSize);

                if (LuminanceOnly || imageBytes.UVRowStride != imageBytes.Width || imageBytes.UVPixelStride != 2)
                {
                    //Not doing any weird values. Y is probably enough.
                    cameraImage.uv = null;
                    return true;
                }
                
                //We expect 2 bytes per pixel, interleaved U/V, with 2x2 subsampling
                bufferSize = imageBytes.Width * imageBytes.Height / 2;
                if(cameraImage.uv == null || cameraImage.uv.Length != bufferSize)
                    cameraImage.uv = new byte[bufferSize];

                //Because U and V planes are returned seperately, while remote expects interleaved U/V
                //same as ARKit, we copy buffers ourselves
                unsafe
                {
                    fixed (byte* uvPtr = cameraImage.uv)
                    {
                        byte* UV = uvPtr;
                        
                        byte* U = (byte*) imageBytes.U.ToPointer();
                        byte* V = (byte*) imageBytes.V.ToPointer();

                        for (int i = 0; i < bufferSize; i+= 2)
                        {
                            *UV++ = *U;
                            *UV++ = *V;

                            U += imageBytes.UVPixelStride;
                            V += imageBytes.UVPixelStride;
                        }
                    }
                }
                
                return true;
            }
        }


        public override bool TryGetPointCloud(ref PointCloud pointCloud)
        {
            if (Session.Status != SessionStatus.Tracking)
                return false;

            // Fill in the data to draw the point cloud.
            m_TempPointCloud.Clear();
            Frame.PointCloud.CopyPoints(m_TempPointCloud);

            if (m_TempPointCloud.Count == 0)
                return false;

            if (pointCloud.points == null)
                pointCloud.points = new List<Vector3>();

            pointCloud.points.Clear();
            foreach (Vector3 point in m_TempPointCloud)
                pointCloud.points.Add(point);

            return true;
        }

        public override LightEstimate GetLightEstimate()
        {
            if (Session.Status.IsValid() && Frame.LightEstimate.State == LightEstimateState.Valid)
            {
                return new LightEstimate()
                {
                    capabilities = LightEstimateCapabilities.AmbientIntensity,
                    ambientIntensity = Frame.LightEstimate.PixelIntensity
                };
            }
            else
            {
                // Zero initialized means capabilities will be None
                return new LightEstimate();
            }
        }

		public override Matrix4x4 GetDisplayTransform()
		{
			return m_DisplayTransform;
		}

        private void CalculateDisplayTransform()
        {
            var cosTheta = 1f;
            var sinTheta = 0f;

            switch (Screen.orientation)
            {
                case ScreenOrientation.Portrait:
                    cosTheta = 0f;
                    sinTheta = -1f;
                    break;
                case ScreenOrientation.PortraitUpsideDown:
                    cosTheta = 0f;
                    sinTheta = 1f;
                    break;
                case ScreenOrientation.LandscapeLeft:
                    cosTheta = 1f;
                    sinTheta = 0f;
                    break;
                case ScreenOrientation.LandscapeRight:
                    cosTheta = -1f;
                    sinTheta = 0f;
                    break;
            }

            m_DisplayTransform.m00 = cosTheta;
            m_DisplayTransform.m01 = sinTheta;
            m_DisplayTransform.m10 = sinTheta;
            m_DisplayTransform.m11 = -cosTheta;
        }

        public override void SetupCamera(Camera camera)
        {
            m_BackgroundRenderer = new ARBackgroundRenderer();
            m_BackgroundRenderer.backgroundMaterial = Resources.Load("Materials/ARBackground", typeof(Material)) as Material;
            m_BackgroundRenderer.camera = camera;
        }

        public override void UpdateCamera(Camera camera)
        {
            if (Screen.orientation != m_CachedScreenOrientation)
            {
                CalculateDisplayTransform();
                m_CachedScreenOrientation = Screen.orientation;
            }

            if (!m_BackgroundRendering || Frame.CameraImage.Texture == null)
                return;

            const string mainTexVar = "_MainTex";
            const string topLeftRightVar = "_UvTopLeftRight";
            const string bottomLeftRightVar = "_UvBottomLeftRight";

            m_BackgroundRenderer.backgroundMaterial.SetTexture(mainTexVar, Frame.CameraImage.Texture);

            var uvQuad = Frame.CameraImage.DisplayUvCoords;

            m_BackgroundRenderer.backgroundMaterial.SetVector(topLeftRightVar,
                new Vector4(uvQuad.TopLeft.x, uvQuad.TopLeft.y, uvQuad.TopRight.x, uvQuad.TopRight.y));
            m_BackgroundRenderer.backgroundMaterial.SetVector(bottomLeftRightVar,
                new Vector4(uvQuad.BottomLeft.x, uvQuad.BottomLeft.y, uvQuad.BottomRight.x, uvQuad.BottomRight.y));

            camera.projectionMatrix = Frame.CameraImage.GetCameraProjectionMatrix(
                camera.nearClipPlane, camera.farClipPlane);

        }

        private bool PlaneUpdated(DetectedPlane detectedPlanep, BoundedPlane bp)
        {
            var tpExtents = new Vector2(detectedPlanep.ExtentX, detectedPlanep.ExtentZ);
            var extents = Vector2.Distance(tpExtents, bp.extents) > 0.005f;
            var rotation = detectedPlanep.CenterPose.rotation != bp.rotation;
            var position = Vector2.Distance(detectedPlanep.CenterPose.position, bp.center) > 0.005f;
            return (extents || rotation || position);
        }

        public override void Update()
        {
            if (m_ARCoreSession == null)
                return;

            AsyncTask.OnUpdate();

            if (Session.Status != SessionStatus.Tracking)
                return;

            if(m_ARCoreSessionConfig.PlaneFindingMode != DetectedPlaneFindingMode.Disabled)
            {
                Session.GetTrackables<DetectedPlane>(m_DetectedPlaneBuffer, TrackableQueryFilter.All);
                foreach (var detectedPlane in m_DetectedPlaneBuffer)
                {
                    BoundedPlane boundedPlane;
                    if (m_DetectedPlanes.TryGetValue(detectedPlane, out boundedPlane))
                    {
                        // remove any subsumed planes
                        if (detectedPlane.SubsumedBy != null)
                        {
                            OnPlaneRemoved(boundedPlane);
                            m_DetectedPlanes.Remove(detectedPlane);
                        }
                        // update any planes with changed extents
                        else if (PlaneUpdated(detectedPlane, boundedPlane))
                        {
                            boundedPlane.center = detectedPlane.CenterPose.position;
                            boundedPlane.rotation = detectedPlane.CenterPose.rotation;
                            boundedPlane.extents.x = detectedPlane.ExtentX;
                            boundedPlane.extents.y = detectedPlane.ExtentZ;
                            m_DetectedPlanes[detectedPlane] = boundedPlane;
                            OnPlaneUpdated(boundedPlane);
                        }
                    }
                    // add any new planes
                    else
                    {
                        boundedPlane = new BoundedPlane()
                        {
                            id = Guid.NewGuid().ToString(),
                            center = detectedPlane.CenterPose.position,
                            rotation = detectedPlane.CenterPose.rotation,
                            extents = new Vector2(detectedPlane.ExtentX, detectedPlane.ExtentZ)
                        };

                        m_DetectedPlanes.Add(detectedPlane, boundedPlane);
                        OnPlaneAdded(boundedPlane);
                    }
                }

                // Check for planes that were removed from the tracked plane list
                List<DetectedPlane> planesToRemove = new List<DetectedPlane>();
                foreach (var kvp in m_DetectedPlanes)
                {
                    var detectedPlane = kvp.Key;
                    if (!m_DetectedPlaneBuffer.Exists(x => x == detectedPlane))
                    {
                        OnPlaneRemoved(kvp.Value);
                        planesToRemove.Add(detectedPlane);
                    }
                }

                foreach (var plane in planesToRemove)
                    m_DetectedPlanes.Remove(plane);

            }

            //Update Anchors
            foreach(var anchor in m_Anchors){
                anchor.Key.transform.position = anchor.Value.transform.position;
                anchor.Key.transform.rotation = anchor.Value.transform.rotation;
            }
        }

        public override void ApplyAnchor(ARAnchor arAnchor)
        {
            if (!IsRunning)
                return;
            //Since ARCore wants to create it's own GameObject, we can keep a reference to it and copy its Pose.
            //Not the best, but probably will change when ARCore releases.
            Anchor arCoreAnchor = Session.CreateAnchor(new Pose(arAnchor.transform.position, arAnchor.transform.rotation));
            arAnchor.anchorID = Guid.NewGuid().ToString();
            m_Anchors[arAnchor] = arCoreAnchor;
        }

        public override void DestroyAnchor(ARAnchor arAnchor)
        {
            if (!string.IsNullOrEmpty(arAnchor.anchorID))
            {
                Anchor arCoreAnchor;
                if(m_Anchors.TryGetValue(arAnchor, out arCoreAnchor)){
                    UnityEngine.Object.Destroy(arCoreAnchor);
                    m_Anchors.Remove(arAnchor);
                }

                arAnchor.anchorID = null;

            }
        }
    }
}
