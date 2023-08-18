#ifndef UTIL_H_
#define UTIL_H_

#include <iostream>
#include <sys/stat.h>
#include "datatype/data_structure.cpp"

using namespace std;

struct Helper{
private:
    const float RAD2ANG_RATIO = 3.14159/180;
    static void computeIoU(cv::Rect rect1, cv::Rect rect2, double &result) {
        double rect1Area        = rect1.width * rect1.height;
        double rect2Area        = rect2.width * rect2.height;
        cv::Rect overlapped     = rect1 & rect2;
        double overlappedArea   = overlapped.width * overlapped.height;

        overlappedArea = (overlappedArea < 0)? 0.: overlappedArea;

        result =  overlappedArea/(rect1Area + rect2Area - overlappedArea);
    }

    static void matchingByMatrix(Eigen::MatrixXd associationMatrix,
                          std::map<int, int>& matches,
                          double distThreshold)
    {
        std::cout<<"association matrix " << associationMatrix<<std::endl;

        while(true)
        {

            double maxValue = distThreshold;
            cv::Point maxLoc;
            for(int i =  0; i < associationMatrix.rows(); i++)
            {

                for(int j =  0; j < associationMatrix.cols(); j++)
                {
                    if(associationMatrix(i, j) > maxValue)
                    {

                        maxLoc.x   = i;
                        maxLoc.y   = j;

                        maxValue    = associationMatrix(i, j);
                    }
                }
            }
            if(maxValue <= distThreshold) break;

            matches.insert(std::pair<int, int>(maxLoc.x, maxLoc.y));

            // TODO: Speed up this procedures by avoid searching within eliminated objects.
            // Add zeros for all corresponding rows/cols of the matched objects.
            for(int i =  0; i < associationMatrix.rows(); i++)
            {
                associationMatrix(i, maxLoc.y)    = distThreshold;
            }
            for(int j =  0; j < associationMatrix.cols(); j++)
            {
                associationMatrix(maxLoc.x, j)         = distThreshold;
            }

        }


    }

public:

    inline static bool isFileExists (const std::string& name)
    {
        struct stat buffer;
        return (stat (name.c_str(), &buffer) == 0);
    }

    static void addPretrainedDetection2D(int stream, DataFrame* dataframe, string streamfile)
    {
        ifstream file2D;
        file2D.open("/home/mohamed/Desktop/Thesis Project/3D-MOT-FeatureBased/src/advanced_detection/det_2d/" + streamfile+".txt");

        string line2D;
        bool found = false;

        while(getline(file2D, line2D))
        {

            istringstream record(line2D);

            string data2D[7];

            for(int i = 0; i < 7; i++)std::getline(record, data2D[i],',');

            if(std::stoi(data2D[0]) == stream)
            {


                found = true;
                if(std::stof(data2D[5]) < 0.45) continue;

                //TrackRCNN ++++
                auto obj = new Object();
                obj->_2dBox = new Box2D();

                obj->_2dBox->boundingBox.x        = std::stoi(data2D[1]);
                obj->_2dBox->boundingBox.y        = std::stoi(data2D[2]);
                obj->_2dBox->boundingBox.width    = abs( std::stoi(data2D[1]) - std::stoi(data2D[3]));
                obj->_2dBox->boundingBox.height   = abs(std::stoi(data2D[2]) - std::stoi(data2D[4]));
                obj->confidence                   = std::stoi(data2D[5]);
                obj->classId                      = 2;
                obj->_2dBox->observationNumber    = 1;
                obj->_2dBox->found = true;

                dataframe->objects.push_back(obj);
            }
            else
            {
                if(found)
                    break;
            }
        }
        file2D.close();

    }


    static void addPretrainedDetection3D(int stream, DataFrame* dataframe, string streamfile)
    {
        ifstream file3D;
        file3D.open("/home/mohamed/Desktop/Thesis Project/3D-MOT-FeatureBased/src/advanced_detection/det_3d/" + streamfile+".txt");

        string line3D;
        bool found = false;
        double counter = 0;

        std::vector<Object> objects;

        while(getline(file3D, line3D))
        {
            istringstream record(line3D);

            string data3D[18];

            for(int i = 0; std::getline(record, data3D[i],','); i++);
            if(std::stoi(data3D[0]) == stream)
            {

                if(stod(data3D[6]) < 4.) continue; //pointrcnn
                //GNN
                cv::Rect boundingBox;
                boundingBox.x = stod(data3D[2]); // 2
                boundingBox.y = stod(data3D[3]); // 3
                boundingBox.width = abs(boundingBox.x - stod(data3D[4])); // 4
                boundingBox.height = abs(boundingBox.y - stod(data3D[5])); // 5

                Object object;
                object._2dBox = new Box2D();
                object._2dBox->boundingBox = boundingBox;
                object._3dBoxLiDAR = new Box3D();
                object._3dBoxLiDAR->x_min = stod(data3D[12]); // 12
                object._3dBoxLiDAR->z_min = -stod(data3D[11]); // 11
                object._3dBoxLiDAR->y_min = -stod(data3D[10]); // 10

                object._3dBoxLiDAR->x_max =  object._3dBoxLiDAR->x_min + stod(data3D[9]); // 9
                object._3dBoxLiDAR->z_max =  object._3dBoxLiDAR->z_min + stod(data3D[8]); // 8
                object._3dBoxLiDAR->y_max = object._3dBoxLiDAR->y_min +  stod(data3D[7])  ; // 7

                object._3dBoxLiDAR->found = true;
                object._3dBoxLiDAR->observationNumber += 1;

                object.alpha = stof(data3D[14]); //14
                object.rt = stof(data3D[13]); // 13
                object.score = stof(data3D[6]); // 6

                objects.push_back(object);
                found = true;
            }
            else
            {
                if(found)
                    break;
            }
        }
        std::cout<<"fine"<<std::endl;

        file3D.close();

        Eigen::MatrixXd associationMatrix(dataframe->objects.size(), objects.size());
        associationMatrix = associationMatrix.setZero();
        std::cout<<"size : " << objects.size()<<std::endl;
        for(int i = 0; i < dataframe->objects.size(); i++)
        {
            for(int j = 0; j < objects.size(); j++)
            {
                double iou;
                computeIoU(objects.at(j)._2dBox->boundingBox, dataframe->objects.at(i)->_2dBox->boundingBox, iou);
                associationMatrix(i, j) = iou;
                std::cout<<"acc"<<std::endl;
            }
        }
        std::cout<<"out"<<std::endl;
        std::map<int, int> matches;
        matchingByMatrix(associationMatrix, matches, 0.25);

        std::vector<int> matchedObjects;

        for(auto match: matches)
        {
            dataframe->objects.at(match.first)->_3dBoxLiDAR = objects.at(match.second)._3dBoxLiDAR;
            dataframe->objects.at(match.first)->alpha = objects.at(match.second).alpha;
            dataframe->objects.at(match.first)->score = objects.at(match.second).score;
            dataframe->objects.at(match.first)->rt = objects.at(match.second).rt;
            matchedObjects.push_back(match.second);
        }

        for(int i = 0; i < objects.size(); i++)
        {
            bool f = false;
            for(int j = 0 ; j < matchedObjects.size(); j++) if (matchedObjects.at(j) == i) f = true;
            if(f) continue;
            std::cout<<"not matched!!"<<std::endl;
            auto object = new Object();
            object->_3dBoxLiDAR = objects.at(i)._3dBoxLiDAR;
            object->alpha = objects.at(i).alpha;
            object->score = objects.at(i).score;
            object->rt = objects.at(i).rt;
            object->_2dBox = objects.at(i)._2dBox;
            object->_2dBox->found = true;
            object->_2dBox->observationNumber    = 1;

            dataframe->objects.push_back(object);
        }
        std::cout<<"tyio"<<std::endl;
    }


    static void addPretrainedGnn2D3D(int stream, DataFrame* dataframe, string streamfile)
    {
        ifstream file2D;
        file2D.open("/home/mohamed/Desktop/Thesis Project/3D-MOT-FeatureBased/src/advanced_detection/pointgnn/" + streamfile+".txt");

        string line2D;
        bool found = false;
        double counter = 0;
        while(getline(file2D, line2D))
        {

            istringstream record(line2D);

            string data2D[18];

            for(int i = 0; std::getline(record, data2D[i],','); i++);

            if(std::stoi(data2D[0]) == stream)
            {
                found = true;
                auto obj = new Object();
                obj->_2dBox = new Box2D();

                obj->alpha                        = std::stof(data2D[4]);
                obj->_2dBox->boundingBox.x        = std::stoi(data2D[5]);
                obj->_2dBox->boundingBox.y        = std::stoi(data2D[6]);
                obj->_2dBox->boundingBox.width    = abs( std::stoi(data2D[7]) - obj->_2dBox->boundingBox.x);
                obj->_2dBox->boundingBox.height   = abs(std::stoi(data2D[8]) - obj->_2dBox->boundingBox.y);
                obj->classId                      = 2;
                obj->_2dBox->observationNumber    = 1;
                obj->confidence                   = 1;
                if(std::stof(data2D[9]) != -1 and std::stof(data2D[10]) != -1 and std::stof(data2D[11]) != -1)
                {
                    obj->_3dBoxLiDAR = new Box3D();
                    obj->_3dBoxLiDAR->x_min = std::stod(data2D[14]); //z
                    obj->_3dBoxLiDAR->z_min = std::stod(data2D[13]); //y
                    obj->_3dBoxLiDAR->y_min = std::stod(data2D[12]); //x

                    obj->_3dBoxLiDAR->y_max = obj->_3dBoxLiDAR->y_min +  std::stof(data2D[9]);
                    obj->_3dBoxLiDAR->z_max = obj->_3dBoxLiDAR->z_min +  std::stof(data2D[10]);
                    obj->_3dBoxLiDAR->x_max = obj->_3dBoxLiDAR->x_min +  std::stof(data2D[11]);
                    std::cout<<obj->_3dBoxLiDAR->x_max<<", "<<obj->_3dBoxLiDAR->y_max<<", "<<obj->_3dBoxLiDAR->z_max<<", "<<std::endl;
                    int x;
                    std::cin>>x;
                    obj->_3dBoxLiDAR->found = true;
                    obj->_3dBoxLiDAR->observationNumber += 1;
                    obj->rt = std::stof(data2D[15]);
                    obj->score = std::stof(data2D[16]);

                }
                std::cout<<"3d"<<std::endl;
                dataframe->objects.push_back(obj);
            }
            else
            {
                if(found)
                    break;
            }
        }
        file2D.close();
    }


    static void saveTrackingData(int stream, std::vector<Object*> objects, string streamfile)
    {
        ofstream file;
        file.open("/media/nagy/New Volume/Ubuntu/Desktop/Thesis Project/3D-MOT-FeatureBased/src/tracking/"+streamfile+".txt", std::ios_base::app);
        if (!file.is_open())
        {
            std::cout<<"not"<<std::endl;
        }
        for(auto object: objects)
        {
            if(!object->_2dBox)
            {
                continue;
            }
            // Write 2D data
            file << stream <<" "<<object->trackId<<" Car "<<-1<<" "<<-1<<" "<<object->alpha <<" "<< object->_2dBox->boundingBox.x <<" " << object->_2dBox->boundingBox.y << " "<<
                 object->_2dBox->boundingBox.x + object->_2dBox->boundingBox.width<<" " << object->_2dBox->boundingBox.y + object->_2dBox->boundingBox.height << " ";

            if(object->_3dBoxLiDAR)
                file <<object->_3dBoxLiDAR->z_max - object->_3dBoxLiDAR->z_min <<" " << object->_3dBoxLiDAR->y_min - object->_3dBoxLiDAR->y_max <<" "
                     << object->_3dBoxLiDAR->x_min - object->_3dBoxLiDAR->x_max <<" " << object->_3dBoxLiDAR->z_min
                     <<" " << object->_3dBoxLiDAR->y_min<<" " << object->_3dBoxLiDAR->x_min<<" "<<object->rt <<" " << object->score<<"\n";
            else
                file <<-1 <<" " <<-1<<" "
                     << -1 <<" " << -1000
                     <<" " << -1000<<" " << -1000<<" "<<"-10 "<<object->score<<"\n";
        }
        file.close();
    }


};

#endif