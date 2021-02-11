using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SightHandler : MonoBehaviour
{
    // 0 Right spawn, 1 left spawn, 2 right end, 3 left end 4 front spawn, 5 front end
    [SerializeField] Transform[] Waypoints;
    [SerializeField] GameObject Wall, Score;
    private GameObject RightWall, LeftWall, FrontWall;
    [SerializeField] float timeToMove;
    private bool moveWalls;
    private FactoryScore fScore;

    void Start()
    {
        RightWall = Instantiate(Wall, Waypoints[0]);
        RightWall.transform.position = Waypoints[0].position;
        LeftWall = Instantiate(Wall, Waypoints[1]);
        LeftWall.transform.position = Waypoints[1].position;
        FrontWall = Instantiate(Wall, Waypoints[4].position, Quaternion.identity);
        fScore = Score.GetComponent<FactoryScore>();
    }

        

    private IEnumerator MoveOverSeconds(GameObject objectToMove, Vector3 end, float seconds)
    {
        float elapsedTime = 0;
        Vector3 startingPos = objectToMove.transform.position;
        while (elapsedTime < seconds)
        {
            objectToMove.transform.position = Vector3.Lerp(startingPos, end, (elapsedTime / seconds));
            elapsedTime += Time.deltaTime;
            yield return new WaitForEndOfFrame();
        }
        objectToMove.transform.position = end;
    }



    public void MoveWallsToPlayer()
    {
        StartCoroutine(MoveOverSeconds(RightWall, Waypoints[2].position, timeToMove));
        StartCoroutine(MoveOverSeconds(LeftWall, Waypoints[3].position, timeToMove));
        StartCoroutine(MoveOverSeconds(FrontWall, Waypoints[5].position, timeToMove));
    }
}
