#include "../utility/urdf_to_pin.hpp"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

using namespace pinocchio;
using namespace std;

RobotWrapper::RobotWrapper(const std::string & filename, const std::vector<std::string> & , bool verbose)
:m_verbose(verbose)
{
    pinocchio::urdf::buildModel(filename, m_model, m_verbose);
    m_model_filename = filename;
    m_na = nv();
}

const Model & RobotWrapper::model() const { return m_model; }
Model & RobotWrapper::model() { return m_model; }

int RobotWrapper::nq() const { return m_model.nq; }
int RobotWrapper::nv() const { return m_model.nv; }
int RobotWrapper::na() const { return m_na; }


void RobotWrapper::computeAllTerms(Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v) 
{
    m_q = q;
    pinocchio::computeAllTerms(m_model, data, q, v);
    data.M.triangularView<Eigen::StrictlyLower>()
            = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    pinocchio::updateFramePlacements(m_model, data);
    pinocchio::centerOfMass(m_model, data, q, v, Eigen::VectorXd::Zero(nv()));
    pinocchio::ccrba(m_model, data, q, v);
}

const SE3 & RobotWrapper::position(const Data & data, const Model::JointIndex index) const
{
    assert(index<data.oMi.size());
    return data.oMi[index];
}

const Motion & RobotWrapper::velocity(const Data & data, const Model::JointIndex index) const
{
    assert(index<data.v.size());
    return data.v[index];
}

void RobotWrapper::jacobianWorld(const Data & data, const Model::JointIndex index, Data::Matrix6x & J) 
{   
    return pinocchio::getJointJacobian(m_model, data, index, pinocchio::WORLD, J) ;    
}

SE3 RobotWrapper::framePosition(const Data & data, const Model::FrameIndex index) const
{
    assert(index<m_model.frames.size());
    const Frame & f = m_model.frames[index];
    return data.oMi[f.parent].act(f.placement);
}

void RobotWrapper::framePosition(const Data & data, const Model::FrameIndex index, SE3 & framePosition) const
{
    assert(index<m_model.frames.size());
    const Frame & f = m_model.frames[index];
    framePosition = data.oMi[f.parent].act(f.placement);
}

Motion RobotWrapper::frameVelocity(const Data & data, const Model::FrameIndex index) const
{
    assert(index<m_model.frames.size());
    const Frame & f = m_model.frames[index];
    return f.placement.actInv(data.v[f.parent]);
}

void RobotWrapper::frameVelocity(const Data & data, const Model::FrameIndex index, Motion & frameVelocity) const
{
    assert(index<m_model.frames.size());
    const Frame & f = m_model.frames[index];
    frameVelocity = f.placement.actInv(data.v[f.parent]);
}

Motion RobotWrapper::frameAcceleration(const Data & data, const Model::FrameIndex index) const
{
    assert(index<m_model.frames.size());
    const Frame & f = m_model.frames[index];
    return f.placement.actInv(data.a[f.parent]);
}

void RobotWrapper::frameAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const
{
    assert(index<m_model.frames.size());
    const Frame & f = m_model.frames[index];
    frameAcceleration = f.placement.actInv(data.a[f.parent]);
}

Motion RobotWrapper::frameClassicAcceleration(const Data & data, const Model::FrameIndex index) const
{
    assert(index<m_model.frames.size());
    const Frame & f = m_model.frames[index];
    Motion a = f.placement.actInv(data.a[f.parent]);
    Motion v = f.placement.actInv(data.v[f.parent]);
    a.linear() += v.angular().cross(v.linear());
    return a;
}

void RobotWrapper::frameClassicAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const
{
    assert(index<m_model.frames.size());
    const Frame & f = m_model.frames[index];
    frameAcceleration = f.placement.actInv(data.a[f.parent]);
    Motion v = f.placement.actInv(data.v[f.parent]);
    frameAcceleration.linear() += v.angular().cross(v.linear());
}

void RobotWrapper::frameJacobianLocal(Data & data, const Model::FrameIndex index, Data::Matrix6x & J) 
{
    Data::Matrix6x J_tmp(6, this->nv());
    assert(index<m_model.frames.size());

    return pinocchio::getFrameJacobian(m_model, data, index, pinocchio::LOCAL, J) ;
}
