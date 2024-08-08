// using System;
// using System.Collections;
// using System.Linq;
// using RosMessageTypes.Geometry;
// using RosMessageTypes.NiryoMoveit;
// using Unity.Robotics.ROSTCPConnector;
// using Unity.Robotics.ROSTCPConnector.ROSGeometry;
// using UnityEngine;
// using System.Collections.Generic;

// public class MultiMoverRos : MonoBehaviour
// {
//     // Generate random objects
//     public GameObject objectPrefab; // 要生成的对象的Prefab
//     public int numberOfObjects; // 要生成的对象数量
//     public float platformRadius; // 圆形平台的半径
//     private List<GameObject> spawnedObjects = new List<GameObject>();
//     private float placePos = -0.2f;

//     // Hardcoded variables
//     const int k_NumRobotJoints = 6;
//     const float k_JointAssignmentWait = 0.1f;
//     const float k_PoseAssignmentWait = 0.5f;

//     // Variables required for ROS communication
//     [SerializeField]
//     string m_RosServiceName = "niryo_moveit";
//     public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

//     [SerializeField]
//     GameObject m_NiryoOne;
//     public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }

//     // Assures that the gripper is always positioned above the m_Target cube before grasping.
//     readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
//     readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

//     // Articulation Bodies
//     ArticulationBody[] m_JointArticulationBodies;
//     ArticulationBody m_LeftGripper;
//     ArticulationBody m_RightGripper;

//     // ROS Connector
//     ROSConnection m_Ros;



//     /// <summary>
//     ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
//     ///     Find left and right finger joints and assign them to their respective articulation body objects.
//     /// </summary>
//     void Start()
//     {
//         ClearObjects();
//         SpawnObjectsWithinCircle();
//         // Get ROS connection static instance
//         m_Ros = ROSConnection.GetOrCreateInstance();
//         m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

//         m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

//         var linkName = string.Empty;
//         for (var i = 0; i < k_NumRobotJoints; i++)
//         {
//             linkName += SourceDestinationPublisher.LinkNames[i];
//             m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
//         }

//         // Find left and right fingers
//         var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
//         var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

//         m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
//         m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();
//         GenerateRandomly spawner = FindObjectOfType<GenerateRandomly>();
//     }

//     /// <summary>
//     ///     Close the gripper
//     /// </summary>
//     void CloseGripper()
//     {
//         var leftDrive = m_LeftGripper.xDrive;
//         var rightDrive = m_RightGripper.xDrive;

//         leftDrive.target = -0.01f;
//         rightDrive.target = 0.01f;

//         m_LeftGripper.xDrive = leftDrive;
//         m_RightGripper.xDrive = rightDrive;
//     }

//     /// <summary>
//     ///     Open the gripper
//     /// </summary>
//     void OpenGripper()
//     {
//         var leftDrive = m_LeftGripper.xDrive;
//         var rightDrive = m_RightGripper.xDrive;

//         leftDrive.target = 0.01f;
//         rightDrive.target = -0.01f;

//         m_LeftGripper.xDrive = leftDrive;
//         m_RightGripper.xDrive = rightDrive;
//     }

//     /// <summary>
//     ///     Get the current values of the robot's joint angles.
//     /// </summary>
//     /// <returns>NiryoMoveitJoints</returns>
//     NiryoMoveitJointsMsg CurrentJointConfig()
//     {
//         var joints = new NiryoMoveitJointsMsg();

//         for (var i = 0; i < k_NumRobotJoints; i++)
//         {
//             joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
//         }

//         return joints;
//     }

//     /// <summary>
//     ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
//     ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
//     ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
//     ///     execute the trajectories in a coroutine.
//     /// </summary>
//     public void PublishJoints()
//     {
//         var request = new MoverServiceRequest();
//         request.joints_input = CurrentJointConfig();
//         var pickPoseList = new List<PoseMsg>(); // 使用List而不是直接使用数组
//         var placePoseList = new List<PoseMsg>();
        

//         foreach(GameObject obj in spawnedObjects)
//         {
//             placePos = placePos + 0.1f;
//             // Pick Pose
//             var pickPose = new PoseMsg
//             {
//                 position = (obj.transform.position + m_PickPoseOffset).To<FLU>(),

//                 // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
//                 orientation = Quaternion.Euler(90, obj.transform.eulerAngles.y, 0).To<FLU>()
//             };
//             pickPoseList.Add(pickPose);

//             // Place Pose
//             var placePose = new PoseMsg
//             {
//                 position = (new Vector3(0.25f, (obj.transform.position.y), placePos) + m_PickPoseOffset).To<FLU>(),
//                 orientation = m_PickOrientation.To<FLU>()
//             };
//             placePoseList.Add(placePose);
//         }
//         var pickPoseArray = new PoseArrayMsg();
//         pickPoseArray.header.frame_id = "world"; // 设置全局坐标系
//         pickPoseArray.poses = pickPoseList.ToArray();

//         var placePoseArray = new PoseArrayMsg();
//         placePoseArray.header.frame_id = "world"; // 同上
//         placePoseArray.poses = placePoseList.ToArray();

//         request.pick_pose = pickPoseArray;
//         request.place_pose = placePoseArray;
//         m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
//     }

//     void TrajectoryResponse(MoverServiceResponse response)
//     {
//         Debug.Log(response.ToString());
//         if (response.trajectories.Length > 0)
//         {
//             Debug.Log("Trajectory returned.");
            
//             // Debug.Log(response.trajectories.Length);
//             // Debug.Log(response.trajectories[2]);
//             StartCoroutine(ExecuteTrajectories(response));
//         }
//         else
//         {
//             Debug.LogError("No trajectory returned from MoverService.");
//         }
//     }

//     /// <summary>
//     ///     Execute the returned trajectories from the MoverService.
//     ///     The expectation is that the MoverService will return four trajectory plans,
//     ///     PreGrasp, Grasp, PickUp, and Place,
//     ///     where each plan is an array of robot poses. A robot pose is the joint angle values
//     ///     of the six robot joints.
//     ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
//     ///     joint values on the robot.
//     /// </summary>
//     /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
//     /// <returns></returns>
//     IEnumerator ExecuteTrajectories(MoverServiceResponse response)
//     {
//         if (response.trajectories != null)
//         {
//             // For every trajectory plan returned
//             for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
//             {
//                 Debug.Log(poseIndex);
//                 // For every robot pose in trajectory plan
//                 foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
//                 {
//                     var jointPositions = t.positions;
//                     var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

//                     // Set the joint values for every joint
//                     for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
//                     {
//                         var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
//                         joint1XDrive.target = result[joint];
//                         m_JointArticulationBodies[joint].xDrive = joint1XDrive;
//                     }

//                     // Wait for robot to achieve pose for all joint assignments
//                     yield return new WaitForSeconds(k_JointAssignmentWait);
//                 }

//                 // Close the gripper if completed executing the trajectory for the Grasp pose
//                 if (poseIndex % 4 == (int)Poses.Grasp)
//                 {
//                     CloseGripper();
//                 }
//                 if (poseIndex % 4 == (int)Poses.Place)
//                 {
//                     OpenGripper();
//                 }

//                 // Wait for the robot to achieve the final pose from joint assignment
//                 yield return new WaitForSeconds(k_PoseAssignmentWait);
//             }

//             // All trajectories have been executed, open the gripper to place the target cube
//             OpenGripper();
//         }
//     }

//     enum Poses
//     {
//         PreGrasp,
//         Grasp,
//         PickUp,
//         Place
//     }

//     public void SpawnObjectsWithinCircle()
//     {
//         ClearObjects();
//         for (int i = 0; i < numberOfObjects; i++)
//         {
//             float radius = UnityEngine.Random.Range(0.25f, platformRadius);
//             float angle = UnityEngine.Random.Range(0, 2 * Mathf.PI);

//             Vector3 randomPosition = new Vector3(
//                 radius * Mathf.Cos(angle),
//                 0.64f,
//                 radius * Mathf.Sin(angle)
//             ); 

//             GameObject spawnedObject = Instantiate(objectPrefab, randomPosition, Quaternion.identity);
//             spawnedObjects.Add(spawnedObject);
//         }
//     }

//     public void ClearObjects()
//     {
//         foreach (GameObject obj in spawnedObjects)
//         {
//             if (obj != null)
//             {
//                 Destroy(obj);
//             }
//         }
//         spawnedObjects.Clear();
//     }
// }
