/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_INERTIA_H
#define IDYNTREE_SPATIAL_INERTIA_H

#include <iDynTree/Core/SpatialInertiaRaw.h>

namespace iDynTree
{
    class Twist;
    class SpatialAcc;
    class SpatialMomentum;
    class Wrench;

    /**
     * Class representing a spatial inertia
     *
     *
     * Currently this class does not support semantics.
     *
     * \ingroup iDynTreeCore
     */
    class SpatialInertia: public SpatialInertiaRaw
    {
    public:
        SpatialInertia();
        SpatialInertia(const double mass,
                       const PositionRaw & com,
                       const RotationalInertiaRaw & rotInertia);
        SpatialInertia(const SpatialInertiaRaw& other);
        SpatialInertia(const SpatialInertia& other);
        virtual ~SpatialInertia();

        // Operations on SpatialInertia
        static SpatialInertia combine(const SpatialInertia & op1,
                                      const SpatialInertia & op2);

        // Get the SpatialInertia as a 6x6 matrix
        Matrix6x6 asMatrix() const;

        // overloaded operators
        SpatialInertia  operator+(const SpatialInertia& other) const;
        SpatialMomentum operator*(const Twist &other) const;
        Wrench operator*(const SpatialAcc &other) const;
    };
}

#endif