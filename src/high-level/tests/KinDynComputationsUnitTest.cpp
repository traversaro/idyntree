/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"
#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/ForwardKinematics.h>

using namespace iDynTree;

double random_double()
{
    return 1.0*((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

double real_random_double()
{
    return 1.0*((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

int real_random_int(int initialValue, int finalValue)
{
    int length = finalValue - initialValue;
    return initialValue + rand() % length;
}

void setRandomState(iDynTree::KinDynComputations & dynComp)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    Transform    worldTbase;
    Twist        baseVel;
    Vector3 gravity;

    iDynTree::VectorDynSize qj(dofs), dqj(dofs), ddqj(dofs);

    worldTbase = //iDynTree::Transform::Identity();
    iDynTree::Transform(Rotation::RPY(random_double(),random_double(),random_double()),
            Position(random_double(),random_double(),random_double()));

    for(int i=0; i < 3; i++)
    {
        gravity(i) = random_double();
    }

    gravity(2) = 0.0;

    for(int i=0; i < 6; i++)
    {
        baseVel(i) = i; //real_random_double();
    }

    for(size_t dof=0; dof < dofs; dof++)

    {
        qj(dof) = random_double();
        dqj(dof) = random_double();
        ddqj(dof) = random_double();
    }

    bool ok = dynComp.setRobotState(worldTbase,qj,baseVel,dqj,gravity);

    iDynTree::VectorDynSize qj_read(dofs), dqj_read(dofs);
    Vector3 gravity_read;
    Transform    worldTbase_read;
    Twist        baseVel_read;
    dynComp.getRobotState(worldTbase_read, qj_read, baseVel_read, dqj_read, gravity_read);

    ASSERT_EQUAL_VECTOR(qj_read, qj);
    ASSERT_EQUAL_VECTOR(dqj_read, dqj);
    ASSERT_EQUAL_VECTOR(gravity_read, gravity);
    ASSERT_EQUAL_TRANSFORM(worldTbase_read, worldTbase);
    ASSERT_EQUAL_VECTOR(baseVel_read.asVector(), baseVel.asVector());

    ASSERT_EQUAL_DOUBLE(ok,true);
}

void testRelativeTransform(iDynTree::KinDynComputations & dynComp)
{
    using namespace iDynTree;

    size_t frames = dynComp.getNrOfFrames();

    // Create some frames for random consistency checks
    FrameIndex A = getRandomInteger(0,frames-1);
    FrameIndex B = getRandomInteger(0,frames-1);
    FrameIndex C = getRandomInteger(0,frames-1);
    FrameIndex D = getRandomInteger(0,frames-1);
    FrameIndex E = getRandomInteger(0,frames-1);
    FrameIndex F = getRandomInteger(0,frames-1);

    Transform A_B_X_E_F = dynComp.getRelativeTransformExplicit(A,B,C,D)*dynComp.getRelativeTransformExplicit(C,D,E,F);
    Transform A_B_X_E_F_check = dynComp.getRelativeTransformExplicit(A,B,E,F);
    Transform E_F_X_A_B = dynComp.getRelativeTransformExplicit(E,F,A,B);

    ASSERT_EQUAL_TRANSFORM(A_B_X_E_F,A_B_X_E_F_check);
    ASSERT_EQUAL_TRANSFORM(A_B_X_E_F,E_F_X_A_B.inverse());
}

void testAverageVelocityAndTotalMomentumJacobian(iDynTree::KinDynComputations & dynComp)
{
    iDynTree::Twist avgVel;
    iDynTree::SpatialMomentum mom;
    iDynTree::Vector6 avgVelCheck, momCheck;
    iDynTree::VectorDynSize nu(dynComp.getNrOfDegreesOfFreedom()+6);
    dynComp.getModelVel(nu);

    MomentumFreeFloatingJacobian momJac(dynComp.getRobotModel());
    FrameFreeFloatingJacobian    avgVelJac(dynComp.getRobotModel());

    avgVel = dynComp.getAverageVelocity();
    bool ok = dynComp.getAverageVelocityJacobian(avgVelJac);

    ASSERT_IS_TRUE(ok);

    mom = dynComp.getLinearAngularMomentum();
    ok = dynComp.getLinearAngularMomentumJacobian(momJac);

    ASSERT_IS_TRUE(ok);

    toEigen(momCheck) = toEigen(momJac)*toEigen(nu);
    toEigen(avgVelCheck) = toEigen(avgVelJac)*toEigen(nu);

    ASSERT_EQUAL_VECTOR(momCheck,mom.asVector());
    ASSERT_EQUAL_VECTOR(avgVelCheck,avgVel.asVector());
}

inline Eigen::VectorXd toEigen(const Vector6 & baseAcc, const VectorDynSize & jntAccs)
{
    Eigen::VectorXd concat(6+jntAccs.size());
    if( jntAccs.size() > 0 )
    {
        concat << toEigen(baseAcc), toEigen(jntAccs);
    }
    else
    {
        concat = toEigen(baseAcc);
    }
    return concat;
}

inline Eigen::VectorXd toEigen(const FreeFloatingGeneralizedTorques & genForces)
{
    Eigen::VectorXd concat(6+genForces.jointTorques().size());
    // TODO(traversaro) : We should teach toEigen to handle empty matrices correctly?
    // relevant: https://forum.kde.org/viewtopic.php?f=74&t=107974
    if( genForces.jointTorques().size() > 0 )
    {
        concat << toEigen(genForces.baseWrench()), toEigen(genForces.jointTorques());
    }
    else
    {
        concat = toEigen(genForces.baseWrench());
    }
    return concat;
}

// Test different ways of computing inverse dynamics
void testInverseDynamics(KinDynComputations & dynComp)
{
    int dofs = dynComp.getNrOfDegreesOfFreedom();
    iDynTree::Vector6 baseAcc;
    iDynTree::JointDOFsDoubleArray shapeAccs(dynComp.model());

    iDynTree::LinkNetExternalWrenches netExternalWrenches(dynComp.model());
    netExternalWrenches.zero();

    // Go component for component, for simplifyng debugging
    for(int i=0; i < 6+dofs; i++)
    {
        baseAcc.zero();
        shapeAccs.zero();
        if( i < 6 )
        {
            baseAcc(i) = 1.0;
        }
        else
        {
            shapeAccs(i-6) = 1.0;
        }

        FreeFloatingGeneralizedTorques invDynForces(dynComp.model());
        FreeFloatingGeneralizedTorques massMatrixInvDynForces(dynComp.model());

        // Run classical inverse dynamics
        bool ok = dynComp.inverseDynamics(baseAcc,shapeAccs,netExternalWrenches,invDynForces);
        ASSERT_IS_TRUE(ok);

        // Run inverse dynamics with mass matrix
        FreeFloatingMassMatrix massMatrix(dynComp.model());
        ok = dynComp.getFreeFloatingMassMatrix(massMatrix);
        ASSERT_IS_TRUE(ok);

        FreeFloatingGeneralizedTorques invDynBiasForces(dynComp.model());
        ok = dynComp.generalizedBiasForces(invDynBiasForces);
        ASSERT_IS_TRUE(ok);

        VectorDynSize massMatrixInvDynForcesContinuous(6+dofs);
        toEigen(massMatrixInvDynForcesContinuous) = toEigen(massMatrix)*toEigen(baseAcc,shapeAccs) + toEigen(invDynBiasForces);
        toEigen(massMatrixInvDynForces.baseWrench().getLinearVec3()) = toEigen(massMatrixInvDynForcesContinuous).segment<3>(0);
        toEigen(massMatrixInvDynForces.baseWrench().getAngularVec3()) = toEigen(massMatrixInvDynForcesContinuous).segment<3>(3);
        toEigen(massMatrixInvDynForces.jointTorques()) = toEigen(massMatrixInvDynForcesContinuous).segment(6,dofs);

        ASSERT_EQUAL_SPATIAL_FORCE(massMatrixInvDynForces.baseWrench(),invDynForces.baseWrench());
        ASSERT_EQUAL_VECTOR(massMatrixInvDynForces.jointTorques(),invDynForces.jointTorques());

    }
}

void testRelativeJacobians(KinDynComputations & dynComp)
{
    if (dynComp.getNrOfLinks() < 2) return;
    FrameIndex frame = -1;
    FrameIndex refFrame = -1;

    if (dynComp.getNrOfLinks() == 2) {
        frame = 0;
        refFrame = 1;
    } else {
        //Pick two frames at random
        frame = real_random_int(0, dynComp.getNrOfFrames());
        refFrame = -1;
        //be sure to pick two different frames
        do {
            refFrame = real_random_int(0, dynComp.getNrOfFrames());
        } while (refFrame == frame && frame >= 0);
    }

    FrameVelocityRepresentation representation = dynComp.getFrameVelocityRepresentation();
    dynComp.setFrameVelocityRepresentation(MIXED_REPRESENTATION);

    //Compute the relative Jacobian
    iDynTree::MatrixDynSize relativeJacobian(6, dynComp.getNrOfDegreesOfFreedom());
    dynComp.getRelativeJacobian(refFrame, frame, relativeJacobian);

    iDynTree::VectorDynSize qj(dynComp.getNrOfDegreesOfFreedom()), dqj(dynComp.getNrOfDegreesOfFreedom());
    Vector3 gravity;
    dynComp.getRobotState(qj, dqj, gravity);

    Twist relativeVel;
    Eigen::Matrix<double, 6, 1> relativeVelTemp;
    relativeVelTemp = toEigen(relativeJacobian) * toEigen(dqj);
    fromEigen(relativeVel, relativeVelTemp);

    //this velocity depends on where the Jacobian is expressed

    Twist frameVel = dynComp.getFrameVel(frame);
    Twist refFrameVel = dynComp.getFrameVel(refFrame);

    if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION) {
        //Inertial = right trivialized.
        //frameVel is written wrt A
        //refFrameVel is written wrt A
        //relativeJacobian is written wrt refFrame
        Eigen::Matrix<double, 6, 1> temp = toEigen(dynComp.getWorldTransform(refFrame).asAdjointTransform()) * toEigen(relativeVel);
        fromEigen(relativeVel, temp);
    } else if (dynComp.getFrameVelocityRepresentation() == BODY_FIXED_REPRESENTATION) {
        //BODY = left trivialized.
        //frameVel is written wrt frame
        //refFrameVel is written wrt refFrame
        //relativeJacobian is written wrt frame
        //convert refFrameVel to frame
        Eigen::Matrix<double, 6, 1> temp = toEigen(dynComp.getRelativeTransform(frame, refFrame).asAdjointTransform()) * toEigen(refFrameVel);
        fromEigen(refFrameVel, temp);
    } else if (dynComp.getFrameVelocityRepresentation() == MIXED_REPRESENTATION) {
        //MIXED
        //frameVel is written wrt frame, [A]
        //refFrameVel is written wrt refFrame, [A]
        //relativeJacobian is written wrt frame, [refFrame]
        //convert refFrameVel to frame, [A]
        Transform frame_A_H_frame(dynComp.getWorldTransform(frame).getRotation(), Position::Zero());

        //refFrameVel = ref_[A]_v_ref, I want frame_[A]_v_ref. As I do not have an explicit A frame I do the following:
        // frame_[A]_H_frame_[frame] * frame_[frame]_H_ref_[frame] * ref_[frame]_H_ref_[A] * ref_[A]_v_ref
        Eigen::Matrix<double, 6, 1> temp = toEigen((frame_A_H_frame * dynComp.getRelativeTransformExplicit(frame, frame, refFrame, frame) * frame_A_H_frame.inverse()).asAdjointTransform()) * toEigen(refFrameVel);
        fromEigen(refFrameVel, temp);
        //and relativeVel to frame [A]
        temp = toEigen((frame_A_H_frame * dynComp.getRelativeTransformExplicit(frame, frame, frame, refFrame)).asAdjointTransform()) * toEigen(relativeVel);
        fromEigen(relativeVel, temp);
    }

    //now compute the error velocity
    //now they should be expressed in the same frame
    Twist velDifference = frameVel - refFrameVel;
    ASSERT_EQUAL_VECTOR(velDifference, relativeVel);

    dynComp.setFrameVelocityRepresentation(representation);
}

/// This helper functions have been copied from KinDynComputations, copied them in a more appropriate place
typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;
/**
 * Function to convert a body fixed acceleration to a mixed acceleration.
 *
 * TODO refactor in a more general handling of conversion between the three
 * derivative of frame velocities (inertial, body-fixed, mixed) and the sensors
 * acceleration.
 *
 * @return The mixed acceleration
 */
Vector6 convertBodyFixedAccelerationToMixedAcceleration(const SpatialAcc & bodyFixedAcc,
                                                        const Twist & bodyFixedVel,
                                                        const Rotation & inertial_R_body)
{
    Vector6 mixedAcceleration;

    Eigen::Map<const Eigen::Vector3d> linBodyFixedAcc(bodyFixedAcc.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angBodyFixedAcc(bodyFixedAcc.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> linBodyFixedTwist(bodyFixedVel.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angBodyFixedTwist(bodyFixedVel.getAngularVec3().data());

    Eigen::Map<Eigen::Vector3d> linMixedAcc(mixedAcceleration.data());
    Eigen::Map<Eigen::Vector3d> angMixedAcc(mixedAcceleration.data()+3);

    Eigen::Map<const Matrix3dRowMajor> inertial_R_body_eig(inertial_R_body.data());

    // First we account for the effect of linear/angular velocity
    linMixedAcc = inertial_R_body_eig*(linBodyFixedAcc + angBodyFixedTwist.cross(linBodyFixedTwist));

    // Angular acceleration can be copied
    angMixedAcc = inertial_R_body_eig*angMixedAcc;

    return mixedAcceleration;
}

/**
 * Function to convert mixed acceleration to body fixed acceleration
 *
 * TODO refactor in a more general handling of conversion between the three
 * derivative of frame velocities (inertial, body-fixed, mixed) and the sensors
 * acceleration.
 *
 * @return The body fixed acceleration
 */
SpatialAcc convertMixedAccelerationToBodyFixedAcceleration(const Vector6 & mixedAcc,
                                                           const Twist & bodyFixedVel,
                                                           const Rotation & inertial_R_body)
{
    SpatialAcc bodyFixedAcc;

    Eigen::Map<const Eigen::Vector3d> linMixedAcc(mixedAcc.data());
    Eigen::Map<const Eigen::Vector3d> angMixedAcc(mixedAcc.data()+3);

    Eigen::Map<const Eigen::Vector3d> linBodyFixedTwist(bodyFixedVel.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angBodyFixedTwist(bodyFixedVel.getAngularVec3().data());

    Eigen::Map<const Matrix3dRowMajor> inertial_R_body_eig(inertial_R_body.data());

    Eigen::Map<Eigen::Vector3d> linBodyFixedAcc(bodyFixedAcc.getLinearVec3().data());
    Eigen::Map<Eigen::Vector3d> angBodyFixedAcc(bodyFixedAcc.getAngularVec3().data());

    linBodyFixedAcc = inertial_R_body_eig.transpose()*linMixedAcc - angBodyFixedTwist.cross(linBodyFixedTwist);
    angBodyFixedAcc = inertial_R_body_eig.transpose()*angMixedAcc;

    return bodyFixedAcc;
}

/**
 * Function to convert inertial acceleration to body acceleration.
 *
 * TODO refactor in a more general handling of conversion between the three
 * derivative of frame velocities (inertial, body-fixed, mixed) and the sensors
 * acceleration.
 *
 * @return The body fixed acceleration
 */
SpatialAcc convertInertialAccelerationToBodyFixedAcceleration(const Vector6 & inertialAcc,
                                                              const Transform & inertial_H_body)
{
    SpatialAcc inertialAccProperForm;
    fromEigen(inertialAccProperForm,toEigen(inertialAcc));
    return inertial_H_body.inverse()*inertialAccProperForm;
}

void testAbsoluteJacobiansAndFrameBiasAcc(KinDynComputations & dynComp)
{
    const Model& model = dynComp.model();

    LinkIndex link = real_random_int(0, dynComp.getNrOfLinks());

    // Compute a random robot acceleration
    Vector6 baseAcc;
    getRandomVector(baseAcc);
    baseAcc.zero();

    JointDOFsDoubleArray jointAcc(6+dynComp.getNrOfDegreesOfFreedom());
    getRandomVector(jointAcc);

    // Convert
    FreeFloatingAcc robotAcc(model);
    robotAcc.jointAcc() = jointAcc;
    Transform world_H_base;
    JointPosDoubleArray jointPos(model);
    Twist baseVel;
    JointDOFsDoubleArray jointVel(model);
    Vector3 gravity;

    dynComp.getRobotState(world_H_base, jointPos, baseVel, jointVel, gravity);
    FreeFloatingPos robotPos(model);
    FreeFloatingVel robotVel(model);

    robotPos.worldBasePos() = world_H_base;
    robotPos.jointPos() = jointPos;
    robotVel.jointVel() = jointVel;

    if ( dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION )
    {
        robotVel.baseVel() = world_H_base.inverse()*baseVel;
        robotAcc.baseAcc() = convertInertialAccelerationToBodyFixedAcceleration(baseAcc, world_H_base);
    }
    else if (dynComp.getFrameVelocityRepresentation() == MIXED_REPRESENTATION)
    {
        robotVel.baseVel() = world_H_base.getRotation().inverse()*baseVel;
        robotAcc.baseAcc() = convertMixedAccelerationToBodyFixedAcceleration(baseAcc, world_H_base.inverse()*baseVel, world_H_base.getRotation());
    }
    else
    {
        assert(dynComp.getFrameVelocityRepresentation() == BODY_FIXED_REPRESENTATION);
        robotVel.baseVel() = baseVel;
        robotAcc.baseAcc() = SpatialAcc(LinearMotionVector3(baseAcc(0),baseAcc(1),baseAcc(2)),
                                        AngularMotionVector3(baseAcc(3),baseAcc(4),baseAcc(5)));
    }

    // Compute the left-trivialized acceleration of the frame using forward kinematics
    Traversal traversal;
    dynComp.model().computeFullTreeTraversal(traversal, dynComp.model().getLinkIndex(dynComp.getFloatingBase()));
    LinkVelArray linkVels(model);
    LinkAccArray linkAccs(model);
    ForwardVelAccKinematics(dynComp.model(), traversal, robotPos, robotVel, robotAcc, linkVels, linkAccs);

    // Compute the link acceleration back to the original reppresentation
    Vector6 linkAcc;
    linkAcc.zero();

    if (dynComp.getFrameVelocityRepresentation() == BODY_FIXED_REPRESENTATION)
    {
        linkAcc = linkAccs(link).asVector();
    }
    else
    {
        // To convert the twist to a mixed or inertial representation, we need world_H_frame
        Transform world_H_link = dynComp.getWorldTransform(link);

        if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION )
        {
            linkAcc = (world_H_link*linkAccs(link)).asVector();
        }
        else
        {
            // In the mixed case, we need to account for the non-vanishing term related to the
            // derivative of the transform between mixed and body representation
            assert(dynComp.getFrameVelocityRepresentation() == MIXED_REPRESENTATION);
            linkAcc = convertBodyFixedAccelerationToMixedAcceleration(linkAccs(link), linkVels(link),world_H_link.getRotation());
        }
    }

    // Compute absolute jacobian
    iDynTree::FrameFreeFloatingJacobian absJac(model);
    dynComp.getFrameFreeFloatingJacobian(link, absJac);

    // Compute bias acc
    Vector6 biasAcc = dynComp.getFrameBiasAcc(link);

    // Check that the acceleration are consistent
    Vector6 linkAccCheck;
    toEigen(linkAccCheck) = toEigen(absJac).block<6,6>(0,0)*toEigen(baseAcc) + toEigen(absJac).block(0,6,6,model.getNrOfDOFs())*toEigen(jointAcc) + toEigen(biasAcc);

    ASSERT_EQUAL_VECTOR(linkAcc, linkAccCheck);

    return;
}

void testModelConsistency(std::string modelFilePath, const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::KinDynComputations dynComp;

    bool ok = dynComp.loadRobotModelFromFile(modelFilePath);
    ASSERT_IS_TRUE(ok);

    ok = dynComp.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    for(int i=0; i < 5; i++)
    {
        setRandomState(dynComp);
        testRelativeTransform(dynComp);
        testAverageVelocityAndTotalMomentumJacobian(dynComp);
        testInverseDynamics(dynComp);
        testRelativeJacobians(dynComp);
        testAbsoluteJacobiansAndFrameBiasAcc(dynComp);
    }

}

int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Testing file " << std::string(IDYNTREE_TESTS_URDFS[mdl]) <<  std::endl;
        testModelConsistency(urdfFileName,iDynTree::MIXED_REPRESENTATION);
        testModelConsistency(urdfFileName,iDynTree::BODY_FIXED_REPRESENTATION);
        testModelConsistency(urdfFileName,iDynTree::INERTIAL_FIXED_REPRESENTATION);
    }

    return EXIT_SUCCESS;
}
