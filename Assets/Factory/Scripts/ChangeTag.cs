using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChangeTag : MonoBehaviour
{
    // Really bad script since when a tag is changed / renamed nothing works...


    private string previousShape, currentShape;

    public void ChangeTagToFillable(int type)
    {
        switch(type)
        {
            case 0: currentShape = "Circle";
                break;
            case 1: currentShape = "Square";
                break;
            case 2: currentShape = "Triangle";
                break;
        }


        previousShape = currentShape;
        GameObject[] allOfThisShape = GameObject.FindGameObjectsWithTag(currentShape);

        foreach (GameObject a in allOfThisShape)
        {
            a.tag = "FillObject";
        }
    }

    public void ResetTag()
    {
        GameObject[] AllFillables = GameObject.FindGameObjectsWithTag("FillObject");

        foreach (GameObject b in AllFillables)
        {
            b.tag = previousShape;
        }
    }
}
