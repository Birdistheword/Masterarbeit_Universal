using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class RespawnAndDestroy : MonoBehaviour
{
    [SerializeField] Transform parent;
    [SerializeField] List<GameObject> ObjectsInScene = new List<GameObject>();
    [SerializeField] List<GameObject> OriginalPrefabs = new List<GameObject>();
    [SerializeField] List<Transform> TransformsRespawner = new List<Transform>();


    public void RespawnObjects()
    {
        foreach (GameObject obj in ObjectsInScene)
        {
            TransformsRespawner.Add(obj.transform);
            Destroy(obj);
        }

        for (int i = 0; i < OriginalPrefabs.Count; i++)
        {
            GameObject current = Instantiate(OriginalPrefabs[i], TransformsRespawner[i]);
            ObjectsInScene[i] = current;
        }
    }
}
