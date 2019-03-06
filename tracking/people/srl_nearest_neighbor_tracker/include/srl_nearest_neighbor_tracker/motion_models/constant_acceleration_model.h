
/*
implements a piecewise constant Weiner process accelarion model

*/

#include <srl_nearest_neighbor_tracker/motion_models/motion_model.h>
#include <ros/ros.h>

namespace srl_nnt{





class ConstantAccelerationModel: Public MotionModel{
    

public ConstantAccelarationModel

{
    m_A = MotionModelMatrix::Zero(DIM,DIM);
    m_Q = MotionModelMatrix::Zero(DIM,DIM);
}

virtual const MotionModelMatrix& A (const StateVector& x, const double deltaT) {
        m_A = MotionModelMatrix::Identity(DIM,DIM);
        m_A(IDX_X, IDX_VX) = deltaT;
        m_A(IDX_Y, IDX_VY) = deltaT;
        m_A(IDX_X, IDX_AX) = 0.5*(deltaT*deltaT);
        m_A(IDX_Y, IDX_AY) =  m_A(IDX_X, IDX_AX);
        m_A(IDX_VX, IDX_AX) = deltaT;
        m_A(IDX_VY, IDX_AY) =  deltaT;
        return m_A;
    }
 virtual const MotionModelMatrix& getProcessNoiseQ(const double deltaT, const double processNoise)
    {
        m_Q = MotionModelMatrix::Zero(DIM,DIM);
        //According to p.276 inEstimation wit Application to Tracking and Navigation
        m_Q(IDX_X, IDX_X) = (deltaT * deltaT * deltaT* deltaT *deltaT) * processNoise / 20.0;
        m_Q(IDX_Y, IDX_Y) =  m_Q(IDX_X, IDX_X);
        m_Q(IDX_VX, IDX_X) = (deltaT * deltaT * deltaT* deltaT) * processNoise / 8.0;
        m_Q(IDX_X, IDX_VX) = m_Q(IDX_VX, IDX_X);
        m_Q(IDX_Y, IDX_VY) = m_Q(IDX_VX, IDX_X);
        m_Q(IDX_VY, IDX_Y) = m_Q(IDX_VX, IDX_X);
        m_Q(IDX_X, IDX_AX) = (deltaT * deltaT * deltaT) * processNoise / 6.0;
        m_Q(IDX_AX, IDX_X) =  m_Q(IDX_X, IDX_AX);
        m_Q(IDX_Y, IDX_Y) =  m_Q(IDX_X, IDX_AX);
        m_Q(IDX_AY, IDX_AY) =  m_Q(IDX_X, IDX_AX);
        m_Q(IDX_VX, IDX_VX) = (deltaT * deltaT * deltaT) * processNoise / 3.0;
        m_Q(IDX_VY, IDX_VY) =  m_Q(IDX_VX, IDX_VX);
        m_Q(IDX_X, IDX_VX) = (deltaT * deltaT) * processNoise / 2.0;
        m_Q(IDX_VX, IDX_X) = m_Q(IDX_X, IDX_VX);
        m_Q(IDX_VY, IDX_Y) = m_Q(IDX_X, IDX_VX);
        m_Q(IDX_Y, IDX_VY) = m_Q(IDX_X, IDX_VX);
        m_Q(IDX_AX, IDX_AX) = deltaT * processNoise;
        m_Q(IDX_AY, IDX_AY) = m_Q(IDX_AX, IDX_AX);

        return m_Q;
    }

virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix)
    {
        MotionModelMatrix model = matrix;
        removeColumn(model,STATE_OMEGA_IDX);
        removeRow(model,STATE_OMEGA_IDX);
        //ROS_INFO_STREAM(matrix << "\nConverted to motionmodel matrix:\n" << model);
        return model;
    }

    virtual const MotionModelVector convertToMotionModel(const StateVector& vector)
    {
        MotionModelVector model = MotionModelVector::Zero(DIM);
        model.head(STATE_VY_IDX) = vector.head(STATE_VY_IDX);
        model.tail(2) = vector.tail(2);
        //ROS_INFO_STREAM(vector << "\nConverted to motionmodel vector:\n" << model);
        return model;
    }

    virtual const StateMatrix convertToState(const MotionModelMatrix& matrix)
    {
        StateMatrix state = StateMatrix::Identity();
        state.block<4,4>(0,0) = matrix.block<4,4>(0,0);
        state.block<2,4>(STATE_AX_IDX,STATE_X_IDX) = matrix.block<2,4>(IDX_AX,IDX_X);
        state.block<4,2>(STATE_X_IDX,STATE_AX_IDX) = matrix.block<4,2>(IDX_X,IDX_AX);
        state(STATE_AX_IDX,STATE_AX_IDX) = matrix(IDX_AX,IDX_AX);
        state(STATE_AY_IDX,STATE_AY_IDX) = matrix(IDX_AY,IDX_AY);
        //ROS_INFO_STREAM(matrix << "\nConverted to state matrix:\n" << state);
        return state;
    }
    virtual const StateVector convertToState(const MotionModelVector& vector)
    {
        StateVector state = StateVector::Zero();
        state.head(4) = vector.head(4);
        state.tail(2) = vector.tail(2);
        //ROS_INFO_STREAM(vector << "\nConverted to state vector:\n" << state);
        return state;
    }

/// Typedefs for easier readability
    typedef boost::shared_ptr<WienerProcessAccelerationModel> Ptr;
    typedef boost::shared_ptr<const WienerProcessAccelerationModel> ConstPtr;

    /// Return a deep copy of this motion model as a shared pointer
    virtual MotionModel::Ptr deepCopy() {
        WienerProcessAccelerationModel* copy = new WienerProcessAccelerationModel();
        *copy = *this;
        return MotionModel::Ptr(copy);





}










}