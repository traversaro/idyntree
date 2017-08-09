/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Direction.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <Eigen/Dense>

#include <cstdio>
#include <sstream>

namespace iDynTree
{
    Direction::Direction(double x, double y, double z)
    {
        this->m_data[0] = x;
        this->m_data[1] = y;
        this->m_data[2] = z;

        this->Normalize();
    }

    Direction::Direction(const double* in_data, const unsigned int in_size):
                 VectorFixSize< 3 >(in_data,in_size)
    {
        this->Normalize();
    }

    Direction::Direction(const Direction& other):VectorFixSize< int(3) >(other)
    {
        this->m_data[0] = other.m_data[0];
        this->m_data[1] = other.m_data[1];
        this->m_data[2] = other.m_data[2];
    }


    void Direction::setToDefault()
    {
        this->m_data[0] = 1.0;
        this->m_data[1] = 0.0;
        this->m_data[2] = 0.0;
    }

    void Direction::Normalize(double tol)
    {
        Eigen::Map<Eigen::Vector3d> vec(this->m_data);
        double nrm = vec.norm();
        if( nrm < tol )
        {
            this->setToDefault();
        }
        else
        {
            vec.normalize();
        }

        return;
    }

    bool Direction::isParallel(const Direction& otherDirection, double tolerance) const
    {
        // The tolerance should be positive
        assert(tolerance > 0);

        // Compute the difference of the norm
        Eigen::Vector3d diff = toEigen(*this)-toEigen(otherDirection);
        double diffNorm = diff.norm();

        // There are two possibilities for two directions to be parallel
        // either they point in the same direction, or in opposite directions
        if( ( diffNorm < tolerance ) ||
            ( fabs(diffNorm-2) < tolerance ) )
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool Direction::isPerpendicular(const Direction& otherDirection, double tolerance) const
    {
        assert(tolerance > 0);

        double fabsDotProduct = fabs(toEigen(*this).dot(toEigen(otherDirection)));

        if( fabsDotProduct > tolerance )
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    Direction Direction::reverse() const
    {
        return Direction(-this->m_data[0],
                         -this->m_data[1],
                         -this->m_data[2]);
    }


    std::string Direction::toString() const
    {
        std::stringstream ss;

        ss << " x " << this->m_data[0]
        << " y " << this->m_data[1]
        << " z " << this->m_data[2];

        return ss.str();
    }

    std::string Direction::reservedToString() const
    {
        return this->toString();
    }

    Direction Direction::Default()
    {
        Direction res;
        res.setToDefault();
        return res;
    }


}
