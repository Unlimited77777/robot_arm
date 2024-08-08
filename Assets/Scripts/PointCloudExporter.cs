using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class PointCloudExporter : MonoBehaviour
{
    public string filePath = "D:/Research/Unity-Robotics-Hub/tutorials/pick_and_place/Point Cloud/PointCloudData.xyz";

    void Start()
    {
        ExportPointCloud();
    }

    void ExportPointCloud()
    {
        List<Vector3> points = new List<Vector3>();
        foreach (Transform child in transform)
        {
            points.Add(child.position);
        }

        WritePointCloudToFile(points, filePath);
    }

    void WritePointCloudToFile(List<Vector3> points, string path)
    {
        using (StreamWriter writer = new StreamWriter(path))
        {
            foreach (var point in points)
            {
                writer.WriteLine($"{point.x} {point.y} {point.z}");
            }
        }

        Debug.Log($"PointCloud exported to {path}");
    }
}
