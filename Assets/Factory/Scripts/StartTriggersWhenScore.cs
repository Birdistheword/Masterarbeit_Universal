using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StartTriggersWhenScore : MonoBehaviour
{
    FactoryScore fScore;
    private int filledCrates, failedCrates;
    [SerializeField] int[] thresFail, thresFill;
    [SerializeField] GameObject RedPlane;
    private SightHandler Sight;
    private int fTriggerCounter = 0;

    void Start()
    {
        fScore = GameObject.FindObjectOfType<FactoryScore>();
        Sight = GameObject.FindObjectOfType<SightHandler>();
    }

    
    // Handle the different Trigger Thresholds (mainly for failed crates)
    // Making sure they only trigger once with fTriggerCounter
    void Update()
    {
        if(fScore.failedCrates >= thresFail[0] && fTriggerCounter == 0)
        {
            Sight.MoveWallsToPlayer();
            fTriggerCounter++;
        }

        if (fScore.failedCrates >= thresFail[1] && fTriggerCounter == 1)
        {
            RedPlane.SetActive(true);
            fTriggerCounter++;
        }

        

        /*if (fScore.filledCrates > thresFill[0])
        {
            // Trigger Successfull Crates 1
        }*/
        
    }
}
