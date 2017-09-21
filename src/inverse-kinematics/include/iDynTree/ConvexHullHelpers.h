/*!
 * @file  ConvexHullHelpers.h
 * @author Silvio Traversaro
 * @copyright 2017 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2017
 *
 */


#ifndef IDYNTREE_CONVEXHULLHELPERS_H
#define IDYNTREE_CONVEXHULLHELPERS_H

#include <string>
#include <vector>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>


namespace iDynTree
{
    /**
     * Class representing a 2D Polygon expressed in the 3D space.
     *
     * A poligon is a geomtric object consisting of a number of points (called vertices) and an equal number of line segments (called sides),
     *  namely a cyclically ordered set of points in a plane, with no three successive points collinear, together with the line segments
     *  joining consecutive pairs of the points. In other words, a polygon is closed broken line lying in a plane.
     */
    class Polygon
    {
    public:
        std::vector<Position> m_vertices;

        /**
         * Default constructor: build an invalid polygon without any vertex.
         */
        Polygon();

        /**
         * Set the number of vertices (the vertices can then be accessed with the operator()
         */
        void setNrOfVertices(size_t size);

        /**
         * Get the number of vertices in the Polygon
         * @return the number of vertices
         */
        size_t getNrOfVertices() const;

        /**
         * Check if a polygon is valid.
         *
         * The condition for the validity of the polygon are:
         * It has at least three points.
         *
         * @return true if is valid, false otherwise.
         */
        bool isValid() const;

        /**
         * Apply a transform on the polygon.
         * @return the transformed polygon.
         */
        Polygon applyTransform(const Transform & newFrame_X_oldFrame) const;

        Position& operator()(const size_t idx);
        const Position & operator()(const size_t idx) const;

        /**
         * Return the polygon of a rectangle on the XY plane given
         * the front (x positive), back (x negative), left (y positive) and right (y negative) offsets.
         *
         */
        static Polygon XYRectangleFromOffsets(const double front, const double back, const double left, const double right);
    };

    /**
     * Class representing a 2D Polygon expressed in the 2D space.
     *
     */
    class Polygon2D
    {
    public:
        std::vector<Vector2> m_vertices;

        /**
         * Default constructor: build an invalid polygon without any vertex.
         */
        Polygon2D();

        /**
         * Set the number of vertices.
         *
         * The vertices can then be accessed with operator() .
         */
        void setNrOfVertices(size_t size);

        /**
         * \brief Get the number of vertices in the Polygon
         * @return the number of vertices
         */
        size_t getNrOfVertices() const;

        /**
         * Check if a polygon is valid.
         *
         * The condition for the validity of the polygon are:
         * It has at least three points.
         *
         * @return true if is valid, false otherwise.
         */
        bool isValid() const;

        Vector2& operator()(const size_t idx);
        const Vector2 & operator()(const size_t idx) const;

        /**
         * Compute an inward offsetted polygon using a simple algorithm, that only works for convex polygons.
         *
         * This routine computes the inward offsetted polygon of the current polygon.
         * The algorithm implementation is simple, and only works if:
         *  * The polygon is convex,
         *  * The offsetted polygon has the same number of vertices of the original polygon.
         *
         *  See https://stackoverflow.com/questions/1109536/an-algorithm-for-inflating-deflating-offsetting-buffering-polygons
         *  for the general problem, that is not addressed by this method.
         *
         *  @param[in]
         *  @return true if everything works correctly, false otherwise.
         */
         bool computeSimpleInwardOffsetOfConvexPolygon(Polygon2D offsetPolygon);
    };

    /**
     * ConvexHullProjectionConstraint helper.
     *
     */
    class ConvexHullProjectionConstraint
    {
        /**
         * Once you compute the projected convex hull, build the A matrix and the b vector such that
         * Ax <= b iff the center of mass projection x is inside the convex hull.
         */
        void buildConstraintMatrix();

        /**
         * Flag to specify if the constraint is active or not.
         */
        bool m_isActive;
    public:
        /**
         * Set if the constraint is active or not.
         */
        void setActive(const bool isActive);

        /**
         * Get if the constraint is active or not.
         * @return true if the constraint is active, false otherwise.
         */
        bool isActive();

        /**
         * Get the number of constraints (i.e. the number rows of the matrix A).
         * @return the number of constraints.
         */
        size_t getNrOfConstraints();

        /**
         * Convex hull expressed in the 2D project constraint plane.
         *
         * This is computed by the buildConvexHull method.
         */
        Polygon2D projectedConvexHull;

        /**
         * A constraint matrix, such that Ax <= b iff the com projection x is in the convex hull.
         */
        MatrixDynSize A;

        /**
         * b vector, such that Ax <= b iff the com projection x is in the convex hull.
         */
        VectorDynSize b;

        /**
         * Projection matrix P,
         * Note that x = P*(c-o), where x is the projection and c is the 3d COM .
         */
        Matrix2x3 P;

        /**
         * Projection matrix 'Pdirection' defined by a given direction.
         * The projection 'x' of a 3D point 'c' along a given vector is obtained as:
         * x = Pdirection*(c-o).
         */
        Matrix2x3 Pdirection;

        /**
         * Matrix obtained multiplyng the matrix A for the matrix P.
         */
        MatrixDynSize AtimesP;

        /**
         * Plane offset o
         * Note that x = P*(c-o), where x is the projection and c is the 3d COM .
         */
        iDynTree::Position o;

        /**
         * \brief Build the projected convex hull.
         *
         * All the polygons are transformed in the the absolute frame, and then they are
         * projected in the projection plane. Once the point have been projected to the 2D plane,
         * a preliminary convex hull is computed using the Monotone Chain algorithm. The preliminary
         * convex hull is transformed in the definitive one by "removing" the specified safety margin.
         *
         * @param[in] projectionPlaneXaxisInAbsoluteFrame X direction of the projection plane, in the absolute frame.
         * @param[in] projectionPlaneYaxisInAbsoluteFrame Y direction of the projection plane, in the absolute frame.
         * @param[in] originOfPlaneInAbsoluteFrame        origin of the projection plane, in the absolute frame.
         * @param[in] supportPolygonsExpressedInSupportFrame Vector of the support polygons, expressed in the support frames.
         * @param[in] absoluteFrame_X_supportFrame Vector of the transform between each support frame and the absolute frame.
         * @param[in] safetyMargin the safety margin of the compute convex hull (default: 0.0).
         * @return true if all went well, false otherwise.
         */
        bool buildConvexHull(const Direction projectionPlaneXaxisInAbsoluteFrame,
                             const Direction projectionPlaneYaxisInAbsoluteFrame,
                             const Position originOfPlaneInAbsoluteFrame,
                             const std::vector<Polygon> & supportPolygonsExpressedInSupportFrame,
                             const std::vector<Transform> & absoluteFrame_X_supportFrame,
                             const double safetyMargin = 0.0);

        /**
         * List of support frames.
         */
        std::vector<int> supportFrameIndices;

        /**
         * List of absolue_X_supportFrames
         */
        std::vector<Transform> absoluteFrame_X_supportFrame;

        /**
         * Project a point in the plane of the convex hull.
         * The point is expressed in the absolute frame of the constriant.
         */
        Vector2 project(iDynTree::Position& posIn3dInAbsoluteFrame);

        /**
         * Compute distance of a 2D point from the convex hull.
         * The distance is positive if the point is inside the convex hull,
         * zero if the point is on the boundary of the convex hull,
         * and negative if it is outside of the convex hull.
         */
        double computeMargin(const Vector2& posIn2D);

        /*!
         * Set the projection matrix 'Pdirection' given a desired projection direction.
         *
         * \author Aiko Dinale (29/08/2017)
         *
         * @param direction     vector along which we want to project a point
         */
        void setProjectionAlongDirection(Vector3 direction);

        /*!
         * Project a point along a direction defined by the projection matrix 'Pdirection'
         *
         * \author Aiko Dinale (29/08/2017)
         *
         * @param posIn3dInAbsoluteFrame     a point we want to project
         */
        Vector2 projectAlongDirection(iDynTree::Position& posIn3dInAbsoluteFrame);

    };
}


#endif
