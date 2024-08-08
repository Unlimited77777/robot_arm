using System;
using System.Collections;
using System.IO;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnitySensors.Sensor.LiDAR;
using System.Collections.Generic;

public class MultiMover : MonoBehaviour
{
    public GameObject sensorObject;
    private RaycastLiDARSensor lidarSensor;
    private int judge = 1;
    private int index = 1;
    // Generate random objects
    public GameObject objectPrefab; // 要生成的对象的Prefab
    public int numberOfObjects; // 要生成的对象数量
    public float platformRadius; // 圆形平台的半径
    private List<GameObject> spawnedObjects = new List<GameObject>();
    private float placePos = -0.2f;
    // 储存机器臂关节位置的列表
    private List<double[]> jointPositionsDuringGrasp = new List<double[]>();
    // 点云
    private List<Vector3> pointCloudData = new List<Vector3>();
    private static int folderIndex = 0; // 静态变量来记录文件夹编号

    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_NiryoOne;
    public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;
    private Quaternion p_orientation;
    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;



    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        if (sensorObject != null)
        {
            lidarSensor = sensorObject.GetComponent<RaycastLiDARSensor>();
            if (lidarSensor == null)
            {
                Debug.LogError("RaycastLiDARSensor component is not found on the Sensor object.");
            }
        }
        else
        {
            Debug.LogError("Sensor object is not assigned.");
        }
        ClearObjects();
        SpawnObjectsWithinCircle();
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();
        GenerateRandomly spawner = FindObjectOfType<GenerateRandomly>();
    }

    //注释开始
    /// <summary>
    /// Capture the current joint configuration and store it in the list.
    /// </summary>
    // void CaptureJointPositions()
    // {
    //     var currentJoints = CurrentJointConfig().joints;
    //     jointPositionsDuringGrasp.Add(currentJoints.ToArray());

    //     // Convert joint positions to Vector3 points and add to point cloud data
    //     for (var i = 0; i < currentJoints.Length; i++)
    //     {
    //         float x = m_JointArticulationBodies[i].transform.position.x;
    //         float y = m_JointArticulationBodies[i].transform.position.y;
    //         float z = m_JointArticulationBodies[i].transform.position.z;
    //         pointCloudData.Add(new Vector3(x, y, z));
    //     }
    // }

    // /// <summary>
    // /// Writes the joint positions to a file at the end of the grasp sequence.
    // /// </summary>
    // void WriteJointPositionsToFile()
    // {
    //     string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
    //     string filename = $"JointPositions_{timestamp}.txt";
    //     string folderPath = @"D:/Research/Unity-Robotics-Hub/tutorials/pick_and_place/Data/";
    //     string path = Path.Combine(folderPath, filename);
    //     using (StreamWriter sw = File.CreateText(path))
    //     {
    //         foreach (var positions in jointPositionsDuringGrasp)
    //         {
    //             sw.WriteLine(string.Join(", ", positions));
    //         }
    //     }
    //     jointPositionsDuringGrasp.Clear(); // Clear the list after writing to file
    // }

    // void WritePointCloudToFile()
    // {
    //     string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
    //     string filename = $"PointCloud_{timestamp}.xyz";
    //     string folderPath = @"D:/Research/Unity-Robotics-Hub/tutorials/pick_and_place/Point Cloud/";
    //     string path = Path.Combine(folderPath, filename);
    //     using (StreamWriter sw = File.CreateText(path))
    //     {
    //         foreach (var point in pointCloudData)
    //         {
    //             sw.WriteLine($"{point.x} {point.y} {point.z}");
    //         }
    //     }
    //     pointCloudData.Clear(); // Clear the list after writing to file
    // }
    //注释结束

    // 收集物体位置并转换为点云数据
    // 收集物体的网格顶点并转换为点云数据
    // 待删
    // void CaptureObjectMeshes()
    // {
    //     foreach (var obj in spawnedObjects)
    //     {
    //         if (obj != null)
    //         {
    //             MeshFilter meshFilter = obj.GetComponent<MeshFilter>();
    //             if (meshFilter != null)
    //             {
    //                 Mesh mesh = meshFilter.mesh;
    //                 if(mesh.isReadable)
    //                 {
    //                     Vector3[] vertices = mesh.vertices;

    //                     foreach (var vertex in vertices)
    //                     {
    //                         // 将本地坐标转换为世界坐标
    //                         Vector3 worldPoint = obj.transform.TransformPoint(vertex);
    //                         pointCloudData.Add(worldPoint);
    //                     }
    //                 }
    //                 else
    //                 {
    //                     Debug.LogError($"Mesh on object {obj.name} is not readable. Enable 'Read/Write Enabled' in import settings.");
    //                 }
    //             }
    //         }
    //     }
    // }


    // 将点云数据写入文件
    //待删
    // void WritePointCloudToFile()
    // {
    //     string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
    //     string filename = $"PointCloud_{timestamp}.xyz";
    //     string folderPath = @"D:/Research/Unity-Robotics-Hub/tutorials/pick_and_place/Point Cloud/";
    //     string path = Path.Combine(folderPath, filename);
    //     using (StreamWriter sw = File.CreateText(path))
    //     {
    //         foreach (var point in pointCloudData)
    //         {
    //             sw.WriteLine($"{point.x} {point.y} {point.z}");
    //         }
    //     }
    //     pointCloudData.Clear(); // Clear the list after writing to file
    // }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints(GameObject obj)
    {
        placePos = placePos + 0.1f;
        judge = 0;
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();
        // if(obj.transform.position.x <= 0.17f && obj.transform.position.x >= -0.17f && obj.transform.position.z < 0){
        //     Debug.Log("11111111111");
        //     p_orientation = m_PickOrientation;
        // }else{
        //     Debug.Log("222222222222");
        //     p_orientation = Quaternion.Euler(90, obj.transform.eulerAngles.y, 0);
        // }
        Vector3 targetDirection = obj.transform.position - m_NiryoOne.transform.position; // 目标方向 Target direction
        Debug.Log(targetDirection);
        targetDirection.y = 0; // 保持水平方向，忽略高度差 Keep horizontal and ignore the height difference

        // 计算目标方向相对于全局北（或Z轴正方向）的角度 Calculate the Angle of the target direction with respect to the global north (or positive Z-axis)
        float angleToTarget = Mathf.Atan2(targetDirection.x, targetDirection.z) * Mathf.Rad2Deg;
        Debug.Log(angleToTarget);
        // 根据object处于的区间，调整传入ROS的orientation Adjust the orientation of the passed ROS according to the interval the object is in
        if (angleToTarget < -45 && angleToTarget >= -135){
            angleToTarget = 270;
        } else if(angleToTarget > 135 || angleToTarget < -135){
            angleToTarget = 180;
        } else if(angleToTarget > 45 && angleToTarget <= 135){
            angleToTarget = 90;
        } else{
            angleToTarget = 0;
        }
        Debug.Log(angleToTarget);
        p_orientation = Quaternion.Euler(90, angleToTarget, 0);
        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (obj.transform.position + m_PickPoseOffset).To<FLU>(),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = p_orientation.To<FLU>()
        };
        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (new Vector3(0.25f, (obj.transform.position.y), placePos) + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };
        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    public void Excute(){
        StartCoroutine(ExcuteIE()); 
    }

    public IEnumerator ExcuteIE(){
        foreach(GameObject obj in spawnedObjects) {
            string folderPath = $"D:/Research/Unity-Robotics-Hub/tutorials/pick_and_place/Point Cloud/point_cloud_{folderIndex++}";
            Directory.CreateDirectory(folderPath);
            //save point cloud
            lidarSensor.ExportPointCloud(folderPath);
            Debug.Log(index);
            PublishJoints(obj);
            yield return new WaitUntil(() => judge == 1);
        }
        
        // 在执行完成后写入点云文件
        //WritePointCloudToFile();
        //点云2
        //待删
        // CaptureObjectMeshes();
        // WritePointCloudToFile();
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        Debug.Log(response.ToString());
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            
            // Debug.Log(response.trajectories.Length);
            // Debug.Log(response.trajectories[2]);
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            Debug.Log("123123123123123");
            Debug.Log(response.trajectories.Length);
            
            
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                Debug.Log(poseIndex);
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }
                    Debug.Log("0000000000");
                    Debug.Log(response.trajectories[poseIndex].joint_trajectory.points.Length);
                    Debug.Log(t);
                    // Capture joint positions
                    // CaptureJointPositions();
                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }
                // Write positions to file after trajectory execution
                // WriteJointPositionsToFile();

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }
                //save point cloud
                lidarSensor.SavePointCloudToPLY();
                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
            judge = 1;
            index++;
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }

    public void SpawnObjectsWithinCircle()
    {
        ClearObjects();
        for (int i = 0; i < numberOfObjects; i++)
        {
            float radius = UnityEngine.Random.Range(0.25f, platformRadius);
            float angle = UnityEngine.Random.Range(0, 2 * Mathf.PI);

            Vector3 randomPosition = new Vector3(
                radius * Mathf.Cos(angle),
                0.64f,
                radius * Mathf.Sin(angle)
            ); 

            GameObject spawnedObject = Instantiate(objectPrefab, randomPosition, Quaternion.identity);
            spawnedObjects.Add(spawnedObject);
        }
    }

    public void ClearObjects()
    {
        foreach (GameObject obj in spawnedObjects)
        {
            if (obj != null)
            {
                Destroy(obj);
            }
        }
        spawnedObjects.Clear();
    }
}
