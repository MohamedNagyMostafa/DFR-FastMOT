//
// Created by nagy on ٣٠‏/٣‏/٢٠٢٢.
//

#include "ObjectAssociation.h"



void ObjectAssociation::runAssociation(std::vector<Object*>& currentObservation,
                                       std::vector<Object*> previousObservation,
                                       std::map<int, int>& matches,
                                       std::vector<AssociationType> associationTypes,
                                       std::vector<double> distThreshold,
                                       bool visual) {

    double distThr;
    Eigen::MatrixXd associationMatrix(currentObservation.size(), previousObservation.size()),
    associationMatrixOut(currentObservation.size(), previousObservation.size());

    associationMatrix = associationMatrix.setZero();
    // Skip if there is no previous objects
    if(previousObservation.size() < 1) return;

    int i = 0;
    // Comparing stage.
    for(AssociationType type: associationTypes) {
        switch (type) {
            case AssociationType::LIDAR_3D_ASSOCIATION:
                distThr = distThreshold.at(i++);
                associationMatrixOut = associationBy3D_KalmanFilterUsingLiDAR(
                        currentObservation,
                        previousObservation,
                        matches,
                        distThr,
                        visual
                );
                associationMatrix +=  associationMatrixOut/distThr;
                std::cout<<"LiDAR 3D Matrix :"<<"\n"<<associationMatrixOut/distThr<<std::endl;

                break;

            case AssociationType::IMAGE_2D_ASSOCIATION:
                distThr = distThreshold.at(i++);

                associationMatrixOut = associationBy2D_KalmanFilter(
                        currentObservation,
                        previousObservation,
                        matches,
                        distThr,
                        visual
                );
//                std::cout<<"Image Matrix :"<<"\n"<<associationMatrixOut<<std::endl;

                associationMatrix += associationMatrixOut/distThr;

                break;

            case AssociationType::IMAGE_2D_ASSOCIATIONIoU:
                distThr = distThreshold.at(i++);

                associationMatrixOut = associationBy2D_KalmanFilterIoU(
                        currentObservation,
                        previousObservation,
                        matches,
                        distThr,
                        visual
                );
                std::cout<<"Image Matrix :"<<"\n"<<associationMatrixOut<<std::endl;

                associationMatrix += associationMatrixOut;

                break;


                associationMatrix += distThr * associationMatrixOut;

                break;
        }
    }

    // Matching stage.
    matchingByMatrix(currentObservation,
                     previousObservation,
                     associationMatrix/(associationTypes.size()),
                     matches,
                     1,
                     visual);

}

void ObjectAssociation::matchingByMatrix(std::vector<Object *> &currentObservation,
                                         std::vector<Object *> previousObservation,
                                         Eigen::MatrixXd associationMatrix,
                                         std::map<int, int>& matches,
                                         double distThreshold,
                                         bool visual)
{

    if(visual)  std::cout<<"Begin association matrix  processing " <<std::endl;


    while(true)
    {
        if(associationMatrix.size() < 1) break;

        // Pick up the minimum value from association matrix.
        double minValue = distThreshold;

        std::cout<<associationMatrix<<std::endl;
        std::ptrdiff_t i, j;
        minValue = associationMatrix.minCoeff(&i,&j);
        auto x  = i;
        auto y  = j;

        if(minValue >= distThreshold) break;
//
        if(visual)  std::cout<<"Associate vehicle id " << previousObservation.at(y)->trackId << " from previous car to "<<
                             currentObservation.at(x)->trackId<<" with distance: " <<std::endl;

        matches.insert(std::pair<int, int>(x, y));

        // TODO: Speed up this procedures by avoid searching within eliminated objects.
        // Add zeros for all corresponding rows/cols of the matched objects.

        associationMatrix.block(0,y,associationMatrix.rows(), 1) = associationMatrix.block(0,y,associationMatrix.rows(), 1).setOnes() *  distThreshold;

        associationMatrix.block(x,0,1, associationMatrix.cols()) =associationMatrix.block(x,0,1, associationMatrix.cols()).setOnes() * distThreshold;


    }
}
/**
 * Associate previous objects with the current detected ones based on distance from estimated state
 * obtained from Kalman Filter for prior objects with current ones.
 * @param currentObservation    current detected objects.
 * @param previousObservation   stored objects in the memory.
 * @param matches               a vector holds index of the association result.
 * @param distThreshold         a maximum distance threshold to associated two objects, if the maximum is
 *                              exceeded, the objects will not be associated.
 * @param visual                to visualize the process.
 */
Eigen::MatrixXd ObjectAssociation::associationBy3D_KalmanFilterUsingLiDAR(std::vector<Object *>& currentObservation,
                                                                          std::vector<Object *> previousObservation,
                                                                          std::map<int, int>& matches,
                                                                          double distThreshold,
                                                                          bool visual) {

    Eigen::MatrixXd associationMatrix(currentObservation.size(), previousObservation.size());
    associationMatrix = associationMatrix.setOnes() * distThreshold;

    for(int currObjectIdx =  0; currObjectIdx < currentObservation.size(); currObjectIdx++)
    {
        for(int prevObjectIdx =  0; prevObjectIdx < previousObservation.size(); prevObjectIdx++)
        {

            auto prevObject = previousObservation.at(prevObjectIdx);
            auto currObject = currentObservation.at(currObjectIdx);

            // Check whether object has 3D bounding box.
            if(!currObject->_3dBoxLiDAR) continue;
            if(!prevObject->_3dBoxLiDAR) continue;

            // Get 3D centroid of previous object from the estimation of KF.

            auto kf3D_EstimationMin = prevObject->_3dBoxLiDAR->stateEstimation3D_min;
            auto kf3D_EstimationMax = prevObject->_3dBoxLiDAR->stateEstimation3D_max;

            double centroidXLoc_preObject = (kf3D_EstimationMin(0) + kf3D_EstimationMax(0))/2.;
            double centroidYLoc_preObject = (kf3D_EstimationMin(1) + kf3D_EstimationMax(1))/2.;
            double centroidZLoc_preObject = (kf3D_EstimationMin(2) + kf3D_EstimationMax(2))/2.;

            // Get 3D centroid of current object from the 3D observation.
            double centroidX_loc_currObject = (currObject->_3dBoxLiDAR->x_min + currObject->_3dBoxLiDAR->x_max)/2.;
            double centroidY_loc_currObject = (currObject->_3dBoxLiDAR->y_min + currObject->_3dBoxLiDAR->y_max)/2.;
            double centroidZ_loc_currObject = (currObject->_3dBoxLiDAR->z_min + currObject->_3dBoxLiDAR->z_max)/2.;

            double distance = sqrt(
                    pow(centroidXLoc_preObject - centroidX_loc_currObject, 2) +
                    pow(centroidYLoc_preObject - centroidY_loc_currObject, 2) +
                    pow(centroidZLoc_preObject - centroidZ_loc_currObject, 2)
            );
//
            if(visual)  cout<<"Object successfully associated, from current frame "<< currentObservation.at(currObjectIdx)->trackId
                            << " to prev frame "<< previousObservation.at(prevObjectIdx)->trackId<<" With distance "<< distance <<std::endl;

//            if(currObject->_3dBoxLiDAR->isFlipped()) continue;
            associationMatrix(currObjectIdx, prevObjectIdx) = (distance > distThreshold)? distThreshold:distance;

        }

    }

    return associationMatrix;
}

Eigen::MatrixXd ObjectAssociation::associationBy2D_KalmanFilter(std::vector<Object *>& currentObservation,
                                                                        std::vector<Object *> previousObservation,
                                                                        std::map<int, int>& matches,
                                                                        double distThreshold,
                                                                        bool visual) {

    Eigen::MatrixXd associationMatrix(currentObservation.size(), previousObservation.size());
    associationMatrix = associationMatrix.setOnes() * distThreshold;

    for(int currObjectIdx =  0; currObjectIdx < currentObservation.size(); currObjectIdx++)
    {
        for(int prevObjectIdx =  0; prevObjectIdx < previousObservation.size(); prevObjectIdx++)
        {

            auto prevObject = previousObservation.at(prevObjectIdx);
            auto currObject = currentObservation.at(currObjectIdx);

            // Check whether object has 3D bounding box.
            if(!currObject->_2dBox) continue;
            if(!prevObject->_2dBox || prevObject->lastObservation() > 30) continue;

            // Get 3D centroid of previous object from the estimation of KF.

            auto kf3D_EstimationTopLeft     = prevObject->_2dBox->stateEstimation2D_topLeft;
            auto kf3D_EstimationBottomRight = prevObject->_2dBox->stateEstimation2D_rightBottom;

            double centroidXLoc_preObject = (kf3D_EstimationTopLeft(0) + kf3D_EstimationBottomRight(0))/2.;
            double centroidYLoc_preObject = (kf3D_EstimationTopLeft(1) + kf3D_EstimationBottomRight(1))/2.;

            // Get 3D centroid of current object from the 3D observation.
            double centroidX_loc_currObject = (currObject->_2dBox->boundingBox.x + currObject->_2dBox->boundingBox.x+ currObject->_2dBox->boundingBox.width )/2.;
            double centroidY_loc_currObject = (currObject->_2dBox->boundingBox.y + currObject->_2dBox->boundingBox.y + currObject->_2dBox->boundingBox.height)/2.;

            double distance = sqrt(
                    pow(centroidXLoc_preObject - centroidX_loc_currObject, 2) +
                    pow(centroidYLoc_preObject - centroidY_loc_currObject, 2)
            );
            if(visual)  cout<<"Estimation loc: (" << kf3D_EstimationTopLeft(0)<<", "<< kf3D_EstimationTopLeft(1) << "), ("<<kf3D_EstimationBottomRight(0) <<","<< kf3D_EstimationBottomRight(1)
            <<"\nLoc: ("<<currObject->_2dBox->boundingBox.x << ", " << currObject->_2dBox->boundingBox.y<<")" << ", (" << currObject->_2dBox->boundingBox.x+ currObject->_2dBox->boundingBox.width
            << ", " << currObject->_2dBox->boundingBox.y + currObject->_2dBox->boundingBox.height<<")"<<std::endl;
            if(visual)  cout<<"Object successfully associated, from current frame "<< currentObservation.at(currObjectIdx)->trackId
                            << " to prev frame "<< previousObservation.at(prevObjectIdx)->trackId<<" With distance "<< distance <<std::endl;

            associationMatrix(currObjectIdx, prevObjectIdx) = (distance > distThreshold)? distThreshold:distance;

        }

    }

    return associationMatrix;
}


Eigen::MatrixXd ObjectAssociation::associationBy2D_KalmanFilterIoU(std::vector<Object *>& currentObservation,
                                                                std::vector<Object *> previousObservation,
                                                                std::map<int, int>& matches,
                                                                double distThreshold,
                                                                bool visual) {

    Eigen::MatrixXd associationMatrix(currentObservation.size(), previousObservation.size());
    associationMatrix = associationMatrix.setOnes();

    for(int currObjectIdx =  0; currObjectIdx < currentObservation.size(); currObjectIdx++)
    {
        for(int prevObjectIdx =  0; prevObjectIdx < previousObservation.size(); prevObjectIdx++)
        {

            auto prevObject = previousObservation.at(prevObjectIdx);
            auto currObject = currentObservation.at(currObjectIdx);

            // Check whether object has 3D bounding box.
            if(!currObject->_2dBox) continue;
            if(!prevObject->_2dBox || prevObject->lastObservation() > 30) continue;

            // Get 3D centroid of previous object from the estimation of KF.

            auto kf3D_EstimationTopLeft     = prevObject->_2dBox->stateEstimation2D_topLeft;
            auto kf3D_EstimationBottomRight = prevObject->_2dBox->stateEstimation2D_rightBottom;
            cv::Rect stateEstimation;
            stateEstimation.x = kf3D_EstimationTopLeft(0);
            stateEstimation.y = kf3D_EstimationTopLeft(1);
            stateEstimation.width = abs(stateEstimation.x - kf3D_EstimationBottomRight(0));
            stateEstimation.height = abs(stateEstimation.y - kf3D_EstimationBottomRight(1));


            // Get 3D centroid of current object from the 3D observation.


            double distance;

            computeIoU(currObject->_2dBox->boundingBox,stateEstimation, distance);

            if(visual)  cout<<"Estimation loc: (" << kf3D_EstimationTopLeft(0)<<", "<< kf3D_EstimationTopLeft(1) << "), ("<<kf3D_EstimationBottomRight(0) <<","<< kf3D_EstimationBottomRight(1)
                            <<"\nLoc: ("<<currObject->_2dBox->boundingBox.x << ", " << currObject->_2dBox->boundingBox.y<<")" << ", (" << currObject->_2dBox->boundingBox.x+ currObject->_2dBox->boundingBox.width
                            << ", " << currObject->_2dBox->boundingBox.y + currObject->_2dBox->boundingBox.height<<")"<<std::endl;
            if(visual)  cout<<"Object successfully associated, from current frame "<< currentObservation.at(currObjectIdx)->trackId
                            << " to prev frame "<< previousObservation.at(prevObjectIdx)->trackId<<" With distance "<< distance <<std::endl;
            associationMatrix(currObjectIdx, prevObjectIdx) = 1- ((distance < 0.05)? 0:distance);

        }

    }
    associationMatrix = associationMatrix;
    return associationMatrix;
}

void ObjectAssociation::computeIoU(cv::Rect rect1, cv::Rect rect2, double &result) {
    double rect1Area        = rect1.width * rect1.height;
    double rect2Area        = rect2.width * rect2.height;
    cv::Rect overlapped     = rect1 & rect2;
    double overlappedArea   = overlapped.width * overlapped.height;

    overlappedArea = (overlappedArea < 0)? 0.: overlappedArea;

    result =  overlappedArea/(rect1Area + rect2Area - overlappedArea);
}

