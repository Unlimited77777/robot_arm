using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GenerateRandomly : MonoBehaviour
{
    public GameObject objectPrefab; // 要生成的对象的Prefab
    public int numberOfObjects; // 要生成的对象数量
    public float platformRadius; // 圆形平台的半径
    private List<GameObject> spawnedObjects = new List<GameObject>();
    public List<Vector3> spawnedObjectPositions = new List<Vector3>();
    public List<Quaternion> spawnedObjectRotations = new List<Quaternion>();

    void Start()
    {
        SpawnObjectsWithinCircle();
    }

    public void SpawnObjectsWithinCircle()
    {
        ClearObjects();
        spawnedObjectPositions.Clear();
        spawnedObjectRotations.Clear();
        for (int i = 0; i < numberOfObjects; i++)
        {
            // 使用极坐标生成点
            float radius = Random.Range(0.25f, platformRadius);
            float angle = Random.Range(0, 2 * Mathf.PI);

            // 将极坐标转换为笛卡尔坐标
            Vector3 randomPosition = new Vector3(
                radius * Mathf.Cos(angle),
                0, // 假设平台在Y=0的高度
                radius * Mathf.Sin(angle)
            ) + transform.position; // 加上平台的中心位置

            // 在计算出的随机位置处生成对象
            GameObject spawnedObject = Instantiate(objectPrefab, randomPosition, Quaternion.identity);
            spawnedObjects.Add(spawnedObject);
        }
        
        
        
    }

    public void Check(){
        foreach(GameObject obj in spawnedObjects){
            spawnedObjectPositions.Add(obj.transform.position);
            spawnedObjectRotations.Add(obj.transform.rotation);
            
        }
        foreach(Vector3 v in spawnedObjectPositions){
            Debug.Log(v);
        }
        foreach(Quaternion q in spawnedObjectRotations){
            Debug.Log(spawnedObjectRotations);
        }
    }

    // 清除所有生成的对象
    public void ClearObjects()
    {
        foreach (GameObject obj in spawnedObjects)
        {
            if (obj != null)
            {
                Destroy(obj);
            }
        }
        spawnedObjects.Clear(); // 清空列表
    }
}
