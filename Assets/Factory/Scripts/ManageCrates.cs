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

    private bool noActiveCrate = true;
    private GameObject currentCrate;
    private ChangeTag TagChange;
    private SignSpawnCircles signScript;
    private FactoryScore score;

    //temp fix
    private bool firstmove = false;

    private float crateSpeed = 0f;
    private Vector3 distanceToEnd;
    private int fillableCollider, fillableAmount, fillableType, lastFillableType, lastFillableCollider, lastFillableAmount;


    void Start()
    {
        // Initialize linked Scripts
        TagChange = GameObject.FindObjectOfType<ChangeTag>();
        signScript = GameObject.FindObjectOfType<SignSpawnCircles>();
        score = GameObject.FindObjectOfType<FactoryScore>();
    }

    
    void FixedUpdate()
    {
        if (!noActiveCrate)
        {
            // TODO: Remove getcomponent from update
            currentCrate.GetComponent<Rigidbody>().MovePosition(currentCrate.transform.localPosition - Vector3.right * Time.deltaTime * crateSpeed);


            distanceToEnd = crateKill.position - currentCrate.transform.position;
            if (distanceToEnd.sqrMagnitude <= .5f)
            {
                CrateReachedEnd();
                print("Crate reached end");
            }
        }
    }


    private void SpawnCrates()
    {
        print("Spawning Crate");
        // Spawn a Crate and assign it to currentCrate
        GameObject newCrate = Instantiate(cratePrefab, crateSpawn.position, Quaternion.identity);
        currentCrate = newCrate;

    
        // Choose Collider to FIll, Amount and Type and send to Sign
        ChooseFillType();
        signScript.SpawnCircleInSection(fillableCollider, fillableAmount, fillableType);
        // Set The right tag for corresponding little shape object
        TagChange.ChangeTagToFillable(fillableType);
    }

    private void CrateReachedEnd()
    {
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
        //Reset tags of fill Objects
        TagChange.ResetTag();
        Destroy(currentCrate);
        noActiveCrate = true;
    }



    public void SetSpawnCrate()
    {
        if (noActiveCrate)
        {
            SpawnCrates();
            print("Spawned crate");
            noActiveCrate = false;
        }
    }

    public void SetStepSpeed(int _step)
    {
        crateSpeed = _step / 4;
        // temp fix for wrong starting pos of lever
        if(!firstmove)
        {
            if (_step.Equals(4)) crateSpeed = 0;
            firstmove = true;
        }

    }







    private void ChooseFillType()
    {
        // watch this number, maybe 4->3 if theres errors (also in line 94)
        fillableCollider = Random.Range(0, 4);

        //Little Hack to not get the same thing twice in a Row
        while (fillableCollider == lastFillableCollider)
        {
            fillableCollider = Random.Range(0, 4);
        }

        fillableAmount = Random.Range(1, 4);

        //Little Hack to not get the same thing twice in a Row
        while (fillableAmount == lastFillableAmount)
        {
            fillableAmount = Random.Range(1, 4);
        }

        // 0 = gears, 1= bananas, 2= phones;
        fillableType = Random.Range(0, 3);

        //Little Hack to not get the same thing twice in a Row
        while (fillableType == lastFillableType)
        {
            fillableType = Random.Range(0, 3);
        }
        if (fillableType == 3) Debug.Log("FillableType Error");

        // Remembering these values for the next Roll ( for the workaround of not getting everything twice)
        lastFillableType = fillableType;
        lastFillableCollider = fillableCollider;
        lastFillableAmount = fillableAmount;

        // Set the chosen Collider to fillable, so that further logic applies
        currentCrate.GetComponent<CrateLogic>().Colliders[fillableCollider].tag = "FillableCollider";

        // This component handles the logic to increase and decrease the amount of right objects in the selected collider
        // and sends that info to the sign
        GameObject.FindGameObjectWithTag("FillableCollider").AddComponent<CountTriggersInCrate>();
        GameObject.FindObjectOfType<CountTriggersInCrate>().SetType(fillableType);
        print("FillableType: " + fillableType);

    }
}
