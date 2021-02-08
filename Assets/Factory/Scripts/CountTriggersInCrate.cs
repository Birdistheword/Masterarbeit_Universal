using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CountTriggersInCrate : MonoBehaviour
{

    private int amountColsInside = 0;
    private int currentType = 0;


    private void OnTriggerEnter(Collider col)
    {
        if (col.gameObject.tag.Equals("FillObject"))
        {
            amountColsInside++;
            GameObject.Find("Sign").GetComponent<SignSpawnCircles>().CorrectFillChangeImage();
        }
    }

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
