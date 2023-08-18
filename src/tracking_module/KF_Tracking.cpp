//
// Created by nagy on ٧‏/٦‏/٢٠٢٢.
//
#include <Eigen/Dense>

#include "KF_Tracking.h"

/**
 * initialize kalman filter gain parameters.
 * @param alpha parameter for object position
 * @param beta  parameter for object velocity
 * @param gamma parameter for object acceleration
 */
KF_Tracking::KF_Tracking(float alpha, float beta, float gamma, float delta_t) {

    // Transition Matrix.
    F_9x9 = Eigen::MatrixXd(9, 9);
    F_6x6 = Eigen::MatrixXd(6, 6);

    F_6x6 <<
            1, 0, delta_t,      0, 0.5 * pow(delta_t,2),               0,
            0, 1,       0, delta_t,            0,   0.5 * pow(delta_t,2),
            0, 0,       1,       0,      delta_t,               0,
            0, 0,       0,       1,            0,         delta_t,
            0, 0,       0,       0,            1,               0,
            0, 0,       0,       0,            0,               1;

    F_9x9 <<
            1,  0,  0,  delta_t,        0,        0,  pow(delta_t, 2)*0.5,                        0,                        0,
            0,  1,  0,        0,  delta_t,        0,                        0,  pow(delta_t, 2)*0.5,                        0,
            0,  0,  1,        0,        0,  delta_t,                        0,                        0,  pow(delta_t, 2)*0.5,

            0,  0,  0,        1,        0,        0,                  delta_t,                        0,                        0,
            0,  0,  0,        0,        1,        0,                        0,                  delta_t,                        0,
            0,  0,  0,        0,        0,        1,                        0,                        0,                  delta_t,

            0,  0,  0,        0,        0,        0,                        1,                        0,                        0,
            0,  0,  0,        0,        0,        0,                        0,                        1,                        0,
            0,  0,  0,        0,        0,        0,                        0,                        0,                        1;


    // Projection Matrix.
    H_9x9 = Eigen::MatrixXd(3,9);
    H_6x6 = Eigen::MatrixXd(2,6);

    H_6x6 <<
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0;

    H_9x9 <<
            1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0;

    // Diagonal Matrix
    K_9x9 = Eigen::MatrixXd(9, 3);
    K_6x6 = Eigen::MatrixXd(6, 2);

    K_6x6 <<
            1, 0,
            0, 1,

            1, 0,
            0, 1,

            1, 0,
            0, 1;

    K_9x9 <<
            1, 0, 0,
            0, 1, 0,
            0, 0, 1,

            1, 0, 0,
            0, 1, 0,
            0, 0, 1,

            1, 0, 0,
            0, 1, 0,
            0, 0, 1;


    // Update Matrix
    G_9x9 = Eigen::MatrixXd(9, 9);
    G_6x6 = Eigen::MatrixXd(6, 6);

    G_6x6 <<
        alpha,          0,             0,            0,                                0,                                 0,
            0,      alpha,             0,            0,                                0,                                 0,
            0,          0,  beta/delta_t,            0,                                0,                                 0,
            0,          0,             0, beta/delta_t,                                0,                                 0,
            0,          0,             0,            0, gamma/(0.5*pow(delta_t,2)),                                 0,
            0,          0,             0,            0,                                0, gamma/(0.5*pow(delta_t, 2));

    G_9x9 <<
            alpha,      0,             0,             0,            0,              0,                                 0,                               0,                                 0,
            0,      alpha,             0,             0,            0,              0,                                 0,                               0,                                 0,
            0,          0,         alpha,             0,            0,              0,                                 0,                               0,                                 0,

            0,          0,             0,  beta/delta_t,            0,              0,                                 0,                               0,                                 0,
            0,          0,             0,             0, beta/delta_t,              0,                                 0,                               0,                                 0,
            0,          0,             0,             0,            0,   beta/delta_t,                                 0,                               0,                                 0,


            0,          0,             0,            0,             0,              0,  gamma/(0.5*pow(delta_t,2)),                                0,                                0,
            0,          0,             0,            0,             0,              0,                                 0, gamma/(0.5*pow(delta_t,2)),                                0,
            0,          0,             0,            0,             0,              0,                                 0,                                0, gamma/(0.5*pow(delta_t,2));

}

KF_Tracking::~KF_Tracking() {

}

/**
 * apply kalman filter in 2D space, a 2D bounding box, on the given object, and update
 * state estimation.
 * @param object object to apply KF.
 */
void KF_Tracking::kalmanFilter2D(Object *object)
{
    // update state.
    if(object->_2dBox->found) {
        // check first observation
        if (object->_2dBox->observationNumber == 1) {
            object->_2dBox->stateEstimation2D_topLeft       = object->_2dBox->stateEstimation2D_topLeft.setZero();
            object->_2dBox->stateEstimation2D_rightBottom   = object->_2dBox->stateEstimation2D_rightBottom.setZero();

            object->_2dBox->stateEstimation2D_topLeft(0) = object->_2dBox->boundingBox.x;
            object->_2dBox->stateEstimation2D_topLeft(1) = object->_2dBox->boundingBox.y;

            object->_2dBox->stateEstimation2D_rightBottom(0) = object->_2dBox->boundingBox.x + object->_2dBox->boundingBox.width;
            object->_2dBox->stateEstimation2D_rightBottom(1) = object->_2dBox->boundingBox.y + object->_2dBox->boundingBox.height;

            object->_2dBox->observationNumber++; // If object appear for one time, no need to come here again.
            return;

        } else {
            Eigen::VectorXd h_min(6), h_max(6), s_min(6), s_max(6);
            Eigen::Vector2d z_min, z_max;

            z_min(0) = object->_2dBox->boundingBox.x;
            z_min(1) = object->_2dBox->boundingBox.y;
            z_max(0) = object->_2dBox->boundingBox.x + object->_2dBox->boundingBox.width;
            z_max(1) = object->_2dBox->boundingBox.y + object->_2dBox->boundingBox.height;

            h_min = z_min - H_6x6 * object->_2dBox->stateEstimation2D_topLeft;
            h_max = z_max - H_6x6 * object->_2dBox->stateEstimation2D_rightBottom;

            s_min = K_6x6 * h_min;
            s_max = K_6x6 * h_max;

            object->_2dBox->stateEstimation2D_topLeft += G_6x6 * s_min;
            object->_2dBox->stateEstimation2D_rightBottom += G_6x6 * s_max;

        }
    }

    // prediction state.
    object->_2dBox->stateEstimation2D_topLeft = F_6x6 * object->_2dBox->stateEstimation2D_topLeft;
    object->_2dBox->stateEstimation2D_rightBottom = F_6x6 * object->_2dBox->stateEstimation2D_rightBottom;

}


/**
 * apply kalman filter in 3D LiDAR space, a 3D bounding box, on the given object, and update
 * state estimation.
 * @param object object to apply KF.
 */
void KF_Tracking::kalmanFilter3D_LiDAR(Object *object)
{
    // update state.
    if(object->_3dBoxLiDAR->found) {
        // check first observation
        if (object->_3dBoxLiDAR->observationNumber == 1) {
            object->_3dBoxLiDAR->stateEstimation3D_min = object->_3dBoxLiDAR->stateEstimation3D_min.setZero();
            object->_3dBoxLiDAR->stateEstimation3D_max = object->_3dBoxLiDAR->stateEstimation3D_max.setZero();

            object->_3dBoxLiDAR->stateEstimation3D_min(0) = object->_3dBoxLiDAR->x_min;
            object->_3dBoxLiDAR->stateEstimation3D_min(1) = object->_3dBoxLiDAR->y_min;
            object->_3dBoxLiDAR->stateEstimation3D_min(2) = object->_3dBoxLiDAR->z_min;

            object->_3dBoxLiDAR->stateEstimation3D_max(0) = object->_3dBoxLiDAR->x_max;
            object->_3dBoxLiDAR->stateEstimation3D_max(1) = object->_3dBoxLiDAR->y_max;
            object->_3dBoxLiDAR->stateEstimation3D_max(2) = object->_3dBoxLiDAR->z_max;
            return;

        } else {

            Eigen::VectorXd h_min(9), h_max(9), s_min(9), s_max(9);
            Eigen::Vector3d z_min, z_max;

            z_min(0) = object->_3dBoxLiDAR->x_min;
            z_min(1) = object->_3dBoxLiDAR->y_min;
            z_min(2) = object->_3dBoxLiDAR->z_min;


            z_max(0) = object->_3dBoxLiDAR->x_max;
            z_max(1) = object->_3dBoxLiDAR->y_max;
            z_max(2) = object->_3dBoxLiDAR->z_max;

            h_min = z_min - H_9x9 * object->_3dBoxLiDAR->stateEstimation3D_min;
            h_max = z_max - H_9x9 * object->_3dBoxLiDAR->stateEstimation3D_max;

            s_min = K_9x9 * h_min;
            s_max = K_9x9 * h_max;

            object->_3dBoxLiDAR->stateEstimation3D_min += G_9x9 * s_min;
            object->_3dBoxLiDAR->stateEstimation3D_max += G_9x9 * s_max;
        }
    }


    // prediction state.
    object->_3dBoxLiDAR->stateEstimation3D_min = F_9x9 * object->_3dBoxLiDAR->stateEstimation3D_min;
    object->_3dBoxLiDAR->stateEstimation3D_max = F_9x9 * object->_3dBoxLiDAR->stateEstimation3D_max;
}

void KF_Tracking::run(std::vector<KF_Tracking::Type> types, std::vector<Object* > objects)
{
    for(auto type: types)
    {
        switch(type)
        {

            case Type::KALMAN_FILTER_3D_LIDAR:
                for(auto object: objects)
                {
                    if(object->_3dBoxLiDAR) kalmanFilter3D_LiDAR(object);
                }
                break;

            case Type::KALMAN_FILTER_2D:
                for(auto object: objects)
                {
                    if(object->_2dBox) kalmanFilter2D(object);
                }
                break;
        }
    }
}
