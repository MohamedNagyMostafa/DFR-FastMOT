//
// Created by nagy on ٢٩‏/٣‏/٢٠٢٢.
//

#include "MemoryManagement.h"


MemoryManagement::MemoryManagement(bool bVisual)
{
    visual = bVisual;
    nextId = 0;
}

void MemoryManagement::updateAssociatedObjects(const std::map<int, int> &matches, std::vector<Object *> &currentObservation)
{
    // Case 1: First frame, no saved objects.
    if(shortTermMemory.empty())
    {
        for(auto object : currentObservation)
        {
                object->trackId = nextId;
                nextId++;

                shortTermMemory.push_back(object);
        }
    }
    else
    {
        // Case 2: Association matched objects
        for(auto match: matches)
        {

            // Current observation
            Object* currentObject   = currentObservation.at(match.first);

            // Previous Observation
            Object* previousObject  = shortTermMemory.at(match.second);

            // Copy previous track id & KF estimation
            currentObject->trackId = (*previousObject).trackId;

            if(currentObject->_3dBoxLiDAR)
            {
                if((*previousObject)._3dBoxLiDAR)
                {
                    currentObject->_3dBoxLiDAR->stateEstimation3D_min = (*previousObject)._3dBoxLiDAR->stateEstimation3D_min;
                    currentObject->_3dBoxLiDAR->stateEstimation3D_max = (*previousObject)._3dBoxLiDAR->stateEstimation3D_max;

                    currentObject->_3dBoxLiDAR->observationNumber += (*previousObject)._3dBoxLiDAR->observationNumber;

                }

                currentObject->_3dBoxLiDAR->found = true;
                currentObject->_3dBoxLiDAR->lastObservation = 0;
            }
            else
            {
                if((*previousObject)._3dBoxLiDAR)
                {
                    currentObject->_3dBoxLiDAR = previousObject->_3dBoxLiDAR;
                    currentObject->_3dBoxLiDAR->found = false;
                }
            }

            if(currentObject->_2dBox)
            {

                if((*previousObject)._2dBox) {
                    currentObject->_2dBox->stateEstimation2D_topLeft = (*previousObject)._2dBox->stateEstimation2D_topLeft;
                    currentObject->_2dBox->stateEstimation2D_rightBottom = (*previousObject)._2dBox->stateEstimation2D_rightBottom;
                    currentObject->_2dBox->observationNumber        += (*previousObject)._2dBox->observationNumber;
                }

                currentObject->_2dBox->found        = true;

                currentObject->_2dBox->lastObservation           = 0;
            }
            else
            {
                if((*previousObject)._2dBox)
                {
                    currentObject->_2dBox = previousObject->_2dBox;
                    currentObject->_2dBox->found = false;
                }
            }


            // Copy setting information

            shortTermMemory.at(match.second) = currentObject;
        }

        // Add objects that doesn't match any
        for(int index = 0; index < currentObservation.size(); index++)
        {
            // To skip associated objects
            bool accept = true;
            if(!matches.empty())
            {
                for(auto match: matches) if (index == match.first){ accept = false; break;}
                if(!accept) continue;
            }
            // Get object information
            Object* object = currentObservation.at(index);

            // Setup tracking id.
            object->trackId = nextId;
            nextId++;
            // Add to memory.
            shortTermMemory.push_back(object);

        }
    }

}

void MemoryManagement::updateObjectStatus()
{
    for(int i =0; i < shortTermMemory.size(); i++)
    {
        auto object = shortTermMemory.at(i);

        if(object->lastObservation() > 30) //|| ((!object->_3dBoxLiDAR || object->_3dBoxLiDAR->isFlipped()) && object->_2dBox->isFlipped()))
        {
            shortTermMemory.erase(shortTermMemory.begin() + i);
            delete object;
            i--;
            continue;
        }
        if(object->_3dBoxIMU)   object->_3dBoxIMU->found    = false;
        if(object->_3dBoxLiDAR)
        {
            object->_3dBoxLiDAR->found  = false;
            object->_3dBoxLiDAR->lastObservation++;
        }
        if(object->_2dBox)
        {
            object->_2dBox->found            = false;
            object->_2dBox->lastObservation++;
        }

    }
}

std::vector<Object *> MemoryManagement::getSavedObjects()
{
    return shortTermMemory;
}

MemoryManagement::~MemoryManagement()
{
    for(auto object: shortTermMemory) delete object;
    for(auto object: longTermMemory) delete object;
}
