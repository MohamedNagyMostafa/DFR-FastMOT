//
// Created by nagy on ٧‏/٦‏/٢٠٢٢.
//

#ifndef TRACKING_3D_KF_TRACKING_H
#define TRACKING_3D_KF_TRACKING_H

#include "../datatype/data_structure.cpp"

class KF_Tracking {
private:

    Eigen::MatrixXd F_9x9, F_6x6; // transition matrix
    Eigen::MatrixXd H_9x9, H_6x6; // projection matrix
    Eigen::MatrixXd K_9x9, K_6x6; // diagonal matrix
    Eigen::MatrixXd G_9x9, G_6x6; // update matrix

    /**
     * apply kalman filter in 2D space, a 2D bounding box, on the given object, and update
     * state estimation.
     * @param object object to apply KF.
     */
    void kalmanFilter2D(Object* object);

    /**
     * apply kalman filter in 3D LiDAR space, a 3D bounding box, on the given object, and update
     * state estimation.
     * @param object object to apply KF.
     */
    void kalmanFilter3D_LiDAR(Object* object);


public:

    /**
     * initialize kalman filter gain parameters and state transition and projection matrices.
     * @param alpha parameter for object position
     * @param beta  parameter for object velocity
     * @param gamma parameter for object acceleration
     */
    KF_Tracking(float alpha, float beta, float gamma, float delta_t);
    ~KF_Tracking();

    /**
     * Types of kalman filter to perform, 2D space (image) or 3D space (LiDAR).
     */
    enum Type
    {
        KALMAN_FILTER_2D,
        KALMAN_FILTER_3D_LIDAR
    };

    /**
     * perform Kalman Filter operations on either 2D, 3D or both.
     * @param types     type of kalman filter to perform, the chosen types should be from @Type.
     * @param objects   objects to perform the operation.
     */
    void run(std::vector<KF_Tracking::Type> types, std::vector<Object *> objects);
};


#endif //TRACKING_3D_KF_TRACKING_H
