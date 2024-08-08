using UnityEngine;
using System.Collections.Generic;
using System.IO;

[RequireComponent(typeof(Camera))]
public class DepthToPointCloud : MonoBehaviour
{
    public Shader depthShader;
    private Camera camera;

    void Start()
    {
        camera = GetComponent<Camera>();
        camera.depthTextureMode = DepthTextureMode.Depth;
        // camera.SetReplacementShader(depthShader, "RenderType");
    }

    void OnDisable()
    {
        camera.ResetReplacementShader();
    }

    void LateUpdate()
    {
        RenderTexture rt = RenderTexture.GetTemporary(camera.pixelWidth, camera.pixelHeight, 16, RenderTextureFormat.ARGB32);
        camera.targetTexture = rt;
        camera.Render();
        RenderTexture.active = rt;

        Texture2D image = new Texture2D(camera.pixelWidth, camera.pixelHeight, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, camera.pixelWidth, camera.pixelHeight), 0, 0);
        image.Apply();

        List<string> pointCloudData = new List<string>();
        for (int y = 0; y < image.height; y++)
        {
            for (int x = 0; x < image.width; x++)
            {
                Color pixel = image.GetPixel(x, y);
                float depth = pixel.r; // Assuming depth is stored in the red channel
                Vector3 point = camera.ScreenToWorldPoint(new Vector3(x, y, depth));
                pointCloudData.Add($"{point.x} {point.y} {point.z}");
            }
        }

        File.WriteAllLines(@"D:\Unity\project\My project (1)\PointCloud.xyz", pointCloudData);
        RenderTexture.active = null;
        camera.targetTexture = null;
        RenderTexture.ReleaseTemporary(rt);
        Destroy(image);
    }
}
