using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SignSpawnCircles : MonoBehaviour
{
    [SerializeField] Image starterImage;
    [SerializeField] Transform[] Spawners;
    [SerializeField] Sprite  cylinder,  banana, phone;

    private Sprite currentSprite;
    private List<Image> currentImageRow = new List<Image>();
    private int currentImageIndex = 0, filledAmount = 0, targetAmount = 0;
    private Image currentImage;

    public bool succesfullyFilled = false;
    
     

    public void SpawnCircleInSection(int index, int amount, int type)
    {
        targetAmount = amount;
        succesfullyFilled = false;
        for(int i = 0; i < amount; i++)
        {

            switch (type)
            {
                // Circle
                case 0:
                    currentSprite = cylinder;
                    break;
                // Square
                case 1:
                    currentSprite = banana;
                    break;
                // Cylinder
                case 2:
                    currentSprite = phone;
                    break;
            }
            starterImage.sprite = currentSprite;

            //Instantiate the Image with the right Sprite
            Image newImage = Instantiate(starterImage, Spawners[index]);

            // Move a bit
            newImage.transform.localPosition = new Vector3(0.25f * i, 0, 0);
            currentImageRow.Add(newImage);
            
        }
        currentImage = currentImageRow[0];

    }
    
    
    public void CorrectFillChangeImage()
    {
        currentImage.color = Color.green;

        if (currentImageIndex<currentImageRow.Count-1)
        {
            currentImageIndex++;
            currentImage = currentImageRow[currentImageIndex];
            
        }
        filledAmount++;
    }

    public void EmptyChangeImage()
    {
        if(currentImageIndex > 0)
        {
            currentImageIndex--;
            currentImage = currentImageRow[currentImageIndex];
        }
        
        //currentImage.sprite = whiteCurrentSprite;
        currentImage.color = Color.white;
        filledAmount--;
    }

    public void KillCircle()
    {
        // Do this ere so I dont have to call it somewhere else
        checkSuccesfullFill();
        foreach (Image img in currentImageRow)
        {
            Destroy(img);
        }

        currentImageRow.Clear();
    }

    private void checkSuccesfullFill()
    {
        //Check for succesfull fill (WARNING: Overfill currently counts as success)
        if (filledAmount == targetAmount) succesfullyFilled = true;
        // and Reset
        filledAmount = 0;
    }

}
