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
    private ChangeTag TagChange;
    private SignSpawnCircles signScript;
    private FactoryScore score;
    private int fillableCollider, fillableAmount, fillableType, lastFillableType, lastFillableCollider, lastFillableAmount;


    void Start()
    {
        StartCoroutine(SpawnCrates());
        TagChange = GameObject.FindObjectOfType<ChangeTag>();
        signScript = GameObject.FindObjectOfType<SignSpawnCircles>();
        score = GameObject.FindObjectOfType<FactoryScore>();
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
        // Spawn and Move
        GameObject newCrate = Instantiate(cratePrefab, crateSpawn.position, Quaternion.identity);
        currentCrate = newCrate;
        moveCurrentCrate = true;


        yield return new WaitForSeconds(4);

    
        // Choose Collider to FIll, Amount and Type and send to Sign
        ChooseFillType();
        signScript.SpawnCircleInSection(fillableCollider, fillableAmount, fillableType);
        // Set The right tag for corresponding little shape object
        TagChange.ChangeTagToFillable(fillableType);

        // Next Status
        moveCurrentCrate = false;
        filledCrate = currentCrate;
        StartCoroutine(MoveCratesToEnd());
    }

    private IEnumerator MoveCratesToEnd()
    {

        // Wait, Move crates to end and destroy
        // TODO: Destroy all objects inside crate (Note: they arent children of the crate, just touching it)
        yield return new WaitForSeconds(10);
        moveFilledCrate = true;
        yield return new WaitForSeconds(4);
        signScript.KillCircle();

        // Managing Score Saving
        if (signScript.succesfullyFilled == true)
        {
            score.filledCrates++;
            Debug.Log("Crate sucessfully filled");
        }

        else
        {
            score.failedCrates++;
            Debug.Log("Crate failed");
        }

        moveFilledCrate = false;

        //Reset tags of fill Objects
        TagChange.ResetTag();
        Destroy(filledCrate);
        StartCoroutine(SpawnCrates());

    }

    private void ChooseFillType()
    {
        // watch this number, maybe 4->3 if theres errors (also in line 94)
        fillableCollider = Random.Range(0, 4);

        //Little Hack to not get the same thing twice in a Row
        while(fillableCollider == lastFillableCollider)
        {
            fillableCollider = Random.Range(0, 4);
        }

        fillableAmount = Random.Range(1, 4);

        //Little Hack to not get the same thing twice in a Row
        while (fillableAmount == lastFillableAmount)
        {
            fillableAmount = Random.Range(1, 4);
        }

        // 0 = circles, 1= squares, 2= Cylinders;
        fillableType = Random.Range(0, 3);

        //Little Hack to not get the same thing twice in a Row
        while (fillableType == lastFillableType)
        {
            fillableType = Random.Range(0, 3);
        }
        if (fillableType == 3) Debug.Log("FillableType Error");

        // Remembering these values for the next Roll
        lastFillableType = fillableType;
        lastFillableCollider = fillableCollider;
        lastFillableAmount = fillableAmount;

        // Set the chosen Collider to fillable, so that further logic applies
        currentCrate.GetComponent<CrateLogic>().Colliders[fillableCollider].tag = "FillableCollider";

        // This component handles the logic to increase and decrease the amount of right objects in the selected collider
        // and sends that info to the sign - so far nothing else
        GameObject.FindGameObjectWithTag("FillableCollider").AddComponent<CountTriggersInCrate>();
        GameObject.FindObjectOfType<CountTriggersInCrate>().SetType(fillableType);


        Debug.Log("Amount to fill: " + fillableAmount);
    }
}
