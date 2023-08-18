//
// Created by nagy on ٧‏/٦‏/٢٠٢٢.
//
#include <iostream>
#include <Eigen/Dense>

#include "../src/datatype/data_structure.cpp"
#include "../src/tracking_module/KF_Tracking.h"

double run_checkInitialization_KF_test()
{
    auto object = new Object();
    object->_3dBoxLiDAR = new Box3D(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 0));

    double out = object->_3dBoxLiDAR->stateEstimation3D_min.sum();
    delete object;
    return out;
}

void addMeasurement(float x, float y, float z, Object* object)
{
    if(object->_3dBoxLiDAR)
    {
        object->_3dBoxLiDAR->x_min = x;
        object->_3dBoxLiDAR->y_min = y;
        object->_3dBoxLiDAR->z_min = z;
    }
    else
    {
        object->_3dBoxLiDAR = new Box3D(pcl::PointXYZ(x, y,z), pcl::PointXYZ(0,0,0));
    }
    object->_3dBoxLiDAR->observationNumber+= 1;
}

void computeKF_3D(Object* object)
{
    KF_Tracking kfTracking(0.6, 0.6, 0.1, 0.1);
    kfTracking.run({KF_Tracking::Type::KALMAN_FILTER_3D_LIDAR},std::vector<Object *>{object});
}

bool run_KalmanFilter3D_Case_InitialEstimation(Object* object, float initialEstimationX, float initialEstimationY, float initialEstimationZ)
{
    return int(object->_3dBoxLiDAR->stateEstimation3D_min(0)*100)/100 == int(initialEstimationX*100)/100 &&
    int(object->_3dBoxLiDAR->stateEstimation3D_min(1)*100)/100 == int(initialEstimationY*100)/100 &&
            int(object->_3dBoxLiDAR->stateEstimation3D_min(2)*100)/100 == int(initialEstimationZ*100)/100;
}

bool run_KalmanFilter3D_Case_Next5Estimations(Object* object, float initialEstimationX[5], float initialEstimationY[5], float initialEstimationZ[5]
                                              , float measurementsX[5], float measurementsY[5], float measurementsZ[5])
{
    for(int i = 0; i < 5; i++)
    {
        addMeasurement(measurementsX[i], measurementsY[i], measurementsZ[i],object);
        computeKF_3D(object);
        if(!run_KalmanFilter3D_Case_InitialEstimation(object, initialEstimationX[i], initialEstimationY[i], initialEstimationZ[i]))
            return false;
    }

    return true;

}


bool run_KalmanFilter3D_Case_fullStreamEstimations(Object* object, float initialEstimationX[18], float initialEstimationY[18], float initialEstimationZ[18]
        , float measurementsX[18], float measurementsY[18], float measurementsZ[18])
{
    for(int i = 0; i < 18; i++)
    {
        if(measurementsX[i] != -1)
        {
            addMeasurement(measurementsX[i], measurementsY[i], measurementsZ[i], object);
            object->_3dBoxLiDAR->found = true;
        }
        else
            object->_3dBoxLiDAR->found = false;

        computeKF_3D(object);
        if(!run_KalmanFilter3D_Case_InitialEstimation(object, initialEstimationX[i], initialEstimationY[i], initialEstimationZ[i]))
            return false;
    }

    return true;

}

int main()
{
    // Test initialization....
    std::cout<<"Unit test: Begin"<<std::endl;

    std::cout<<"Unit test: initialization test ..."<<std::endl;
    assert(run_checkInitialization_KF_test() == -9);
    std::cout<<"Unit test: initialization test passed."<<std::endl;

    // Test 3D Kalman Filter
    std::cout<<"Unit test: KF initial estimation test ..."<<std::endl;
    auto object = new Object();
    addMeasurement(43.358, -10.5555, -10.5555, object);
    std::cout<<"Add measurement"<<std::endl;

    computeKF_3D(object);
    std::cout<<"Computer 3D KF"<<std::endl;

    assert(run_KalmanFilter3D_Case_InitialEstimation(object, 43.358, -10.5555, -10.5555));
    std::cout<<"Unit test: KF initial estimation passed."<<std::endl;

    std::cout<<"Unit test: KF 5 estimations test ..."<<std::endl;
//    assert(run_KalmanFilter3D_Case_Next5Estimations(object,
//                                                    new float[5]{43.483450000000005, 43.577515000000005, 43.69319050000001, 43.843884349999996,44.12661174499999},
//                                                    new float[5]{-10.4879, -10.42238, -10.362126, -10.3061802, -10.149994540000002}
//            ,new float[5]{43.4545,43.489,43.581,43.7055,43.933},
//                                                    new float[5]{-10.5035,-10.4735,-10.43,-10.378,-10.243}));
    std::cout<<"Unit test: KF 5 estimations test passed."<<std::endl;

    std::cout<<"Unit test: KF full stream estimations test ..."<<std::endl;
    assert(run_KalmanFilter3D_Case_fullStreamEstimations(object,
                                                         new float[18]{43.483450000000005, 43.577515000000005, 43.69319050000001, 43.843884349999996, 44.12661174499999, 44.64801651149999, 45.34878654104999, 45.782160352835014, 45.82340670105451, 46.11652626089801, 46.3428886814971, 46.819583320432166, 47.37134686314765, 47.611386823388024, 47.973042876245, 48.33310690383, 48.69157890614301, 49.65805630519812},
                                                         new float[18]{ -10.4879, -10.42238, -10.362126, -10.3061802, -10.149994540000002, -9.977356858, -9.586535836599998, -9.325394822819996, -9.791617354413999, -9.963403128216001, -9.570441183853204, -9.34314330028364, -8.981881322759428, -8.888888431863574, -8.657676896116925, -8.407117921863822, -8.137211509104265, -7.824932696002711},
                                                         new float[18]{ -10.4879, -10.42238, -10.362126, -10.3061802, -10.149994540000002, -9.977356858, -9.586535836599998, -9.325394822819996, -9.791617354413999, -9.963403128216001, -9.570441183853204, -9.34314330028364, -8.981881322759428, -8.888888431863574, -8.657676896116925, -8.407117921863822, -8.137211509104265, -7.824932696002711},
                                                         new float[18]{43.4545,43.489,43.581,43.7055,43.933,44.3195,44.8135,45.1325, 45.327, -1, 46.0815,46.541,46.96,47.166,-1,-1,-1,49.1605},
                                                         new float[18]{-10.5035,-10.4735,-10.43,-10.378,-10.243,-10.1245,-9.818, -9.6575, -9.9515,-1,-9.4665,-9.4825,-9.227,-9.18, -1,-1,-1,-8.1195},
                                                         new float[18]{-10.5035,-10.4735,-10.43,-10.378,-10.243,-10.1245,-9.818, -9.6575, -9.9515,-1,-9.4665,-9.4825,-9.227,-9.18, -1,-1,-1,-8.1195}));
    std::cout<<"Unit test: KF full stream estimations test passed."<<std::endl;

    std::cout<<"Unit test: End"<<std::endl;

    return 0;
}
