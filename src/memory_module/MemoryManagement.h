//
// Created by nagy on ٢٩‏/٣‏/٢٠٢٢.
//

#ifndef TRACKING_3D_MEMORYMANAGEMENT_H
#define TRACKING_3D_MEMORYMANAGEMENT_H

#include <iostream>
#include <random>

#include "../datatype/data_structure.cpp"


class MemoryManagement{

private:
    std::vector<Object*> shortTermMemory;
    std::vector<Object*> longTermMemory;
    int nextId;
    bool visited[100];
    bool visual;

public:
    MemoryManagement(bool bVisual = false);
    ~MemoryManagement();

    std::vector<Object*> getSavedObjects() ;
    void updateAssociatedObjects(const std::map<int, int>& matches, std::vector<Object*>& currentObservation);
    long pickUniqueId();
    void updateObjectStatus();

};


#endif //TRACKING_3D_MEMORYMANAGEMENT_H
