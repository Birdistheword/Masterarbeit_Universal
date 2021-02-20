using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HurricaneVR.Framework.Core;



public class CountTriggersInCrate : MonoBehaviour
{

    private int amountColsInside = 0;
    private int currentType = 0;


    private void OnTriggerEnter(Collider col)
    {
        if (col.gameObject.tag.Equals("FillObject"))
        {
            // count up and update sign
            amountColsInside++;
            GameObject.Find("Sign").GetComponent<SignSpawnCircles>().CorrectFillChangeImage();

            // parent thrown object to crate (for destroy) and make it ungrabbable
            col.transform.parent = gameObject.transform.parent;
            Destroy(col.gameObject.GetComponent<HVRGrabbable>());

            print("Count 1 more trigger in crate");
        }
    }


    // This should now be deprecated but I'll leave it just in case
    private void OnTriggerExit(Collider col)
    {
        if (col.gameObject.tag.Equals("FillObject"))
        {
            amountColsInside--;
            GameObject.Find("Sign").GetComponent<SignSpawnCircles>().EmptyChangeImage();
        }
    }

    public void SetType(int type)
    {
        currentType = type;
    }
}
