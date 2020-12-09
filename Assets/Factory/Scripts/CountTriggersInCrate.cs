using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CountTriggersInCrate : MonoBehaviour
{

    private int amountColsInside = 0;
    


    private void OnTriggerEnter(Collider col)
    {
        if (col.gameObject.tag.Equals("FillObject") && gameObject.tag.Equals("FillableCollider"))
        {
            amountColsInside++;
            GameObject.Find("Sign").GetComponent<SignSpawnCircles>().CorrectFillChangeImage();
            Debug.Log("There are " + amountColsInside + "Balls in the Right Section");
        }
    }

    private void OnTriggerExit(Collider col)
    {
        if (col.gameObject.tag.Equals("FillObject") && gameObject.tag.Equals("FillableCollider"))
        {
            amountColsInside--;
            GameObject.Find("Sign").GetComponent<SignSpawnCircles>().EmptyChangeImage();
            Debug.Log("There are " + amountColsInside + "Balls in the Right Section");
        }
    }
}
