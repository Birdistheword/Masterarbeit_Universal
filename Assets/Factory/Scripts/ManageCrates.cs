using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ManageCrates : MonoBehaviour
{
    [SerializeField] GameObject cratePrefab;
    [SerializeField] Transform crateSpawn;
    [SerializeField] Transform crateKill;
    [SerializeField] Transform crateStation;
    [SerializeField] GameObject sign;

    [SerializeField] float crateMovespeed = 5;

    private bool moveCurrentCrate = false, moveFilledCrate = false, nextCoroutine = true;
    private GameObject currentCrate;
    private GameObject filledCrate;
    private int fillableCollider, fillableAmount;


    void Start()
    {
            StartCoroutine(SpawnCrates());
    }

    
    void FixedUpdate()
    {
        if(moveCurrentCrate)
        {
            //currentCrate.transform.position = Vector3.Lerp(currentCrate.transform.position, crateStation.position, Time.deltaTime * crateMovespeed);
            currentCrate.GetComponent<Rigidbody>().MovePosition(currentCrate.transform.localPosition - Vector3.right * Time.deltaTime * crateMovespeed);
        }

        if (moveFilledCrate)
        {
            //filledCrate.transform.position = Vector3.Lerp(currentCrate.transform.position, crateKill.position, Time.deltaTime * crateMovespeed);
            filledCrate.GetComponent<Rigidbody>().MovePosition(currentCrate.transform.localPosition - Vector3.right * Time.deltaTime * crateMovespeed);
        }

    }

    private IEnumerator SpawnCrates()
    {
        GameObject newCrate = Instantiate(cratePrefab, crateSpawn.position, Quaternion.identity);
        currentCrate = newCrate;
        moveCurrentCrate = true;

        
        


        yield return new WaitForSeconds(4);
        // Choose Collider to Fill and send to Sign
        ChooseFillType();
        sign.GetComponent<SignSpawnCircles>().SpawnCircleInSection(fillableCollider);
        moveCurrentCrate = false;
        filledCrate = currentCrate;
        StartCoroutine(MoveCratesToEnd());
    }

    private IEnumerator MoveCratesToEnd()
    {
        yield return new WaitForSeconds(5);
        moveFilledCrate = true;
        yield return new WaitForSeconds(4);
        sign.GetComponent<SignSpawnCircles>().KillCircle();
        moveFilledCrate = false;
        Destroy(filledCrate);
        StartCoroutine(SpawnCrates());

    }

    private void ChooseFillType()
    {
        fillableCollider = Random.Range(0, 3);
        fillableAmount = Random.Range(1, 2);

        currentCrate.GetComponent<CrateLogic>().Colliders[fillableCollider].tag = "FillableCollider";
        Debug.Log("The fillable collider is: " + currentCrate.GetComponent<CrateLogic>().Colliders[fillableCollider]);
        Debug.Log("Amount to fill: " + fillableAmount);
    }
}
