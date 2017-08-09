/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SUB_MODEL_H
#define IDYNTREE_SUB_MODEL_H

#include <iDynTree/Model/Indices.h>

#include <vector>
#include <string>
#include <cstddef>

namespace iDynTree
{
    class Model;
    class Traversal;
    class JointPosDoubleArray;
    class LinkPositions;
    class IRawVector;

    /**
     * Class representing the decomposition in one model in several submodels.
     *
     * This class is used in algorithms, such as estimation of external wrenches,
     * where a complete model is decomposed in several submodels.
     *
     * For each submodel a iDynTree::Traversal is provided, so that algorithms
     * that are explicitly designed to run on both FullModel Traversal and SubModel
     * traversal can be executed on the SubModel.
     *
     */
    class SubModelDecomposition
    {
    private:
        /**
         * Vector of size nrOfSubModels traversal, one for each submodel.
         */
        std::vector<Traversal *> subModelTraversals;

        /**
         * Vector mapping link index to sub model index.
         */
        std::vector<size_t> link2subModelIndex;

        /**
         * Copy constructor is forbidden
         */
        SubModelDecomposition(const SubModelDecomposition & other);

        /**
         * Copy operator is forbidden
         */
        SubModelDecomposition& operator=(const SubModelDecomposition &other);

    public:
        /**
         * Constructor
         */
        SubModelDecomposition();

        /**
         * Destructor
         */
        ~SubModelDecomposition();

        /**
         * Create a submodel decomposition
         * of a given model with a given full tree traversal,
         * by separating the model across the given joints.
         *
         * @param[in] model the model to split
         * @param[in] traversal the full model traversal to split
         * @param[in] splitJoints the model will be split along
         *                        the joints whose names are in this vector.
         * @return true if all went fine, false otherwise
         */
        bool splitModelAlongJoints(const Model & model,
                                   const Traversal & traversal,
                                   const std::vector<std::string>& splitJoints);

        /**
         * Create a submodel decomposition
         * of a given model with a given full tree traversal,
         * such that the specified links are the basis of the submodel present in the decomposition.
         *
         * @param[in] model the model to split
         * @param[in] traversal the full model traversal to split
         * @param[in] subModelBasis the model will be split in a way such that the specified links will be the submodel bases.
         *
         * The link not specified will be assigned to the submodel for which the number of the DOF between the base and the link
         * are minimal in the model. In case of tie, the submodel with lower link index will choosen.
         *
         * @warning This method assume that no closed loops are present in the Model object.
         * @return true if all went fine, false otherwise
         */
        bool splitModelWithSpecifiedBasesLinks(const Model & model,
                                               const Traversal & traversal,
                                               const std::vector<std::string>& subModelBasis);

        /**
         * Set the number of the submodels in the decomposition.
         */
        void setNrOfSubModels(const size_t nrOfSubModels);

        /**
         * Get the number of submodels in the decomposition.
         */
        size_t getNrOfSubModels() const;

        /**
         * Get the total numer of links in the submodel decomposition.
         */
        size_t getNrOfLinks() const;

        /**
         * Get the traversal of a given submodel
         */
        Traversal & getTraversal(const size_t subModelIndex);

        /**
         * Get the traversal of a given submodel (const version)
         */
        const Traversal & getTraversal(const size_t subModelIndex) const;

        /**
         * Get the subModel to which a given links belongs.
         */
        size_t getSubModelOfLink(const LinkIndex & link) const;

        /**
         * Get the subModel to which a given frame belongs.
         */
        size_t getSubModelOfFrame(const Model & model, const FrameIndex& frame) const;

    };

    /**
     * Helper loop to compute the position of each link
     * wrt to the frame of the subModel base.
     *
     * @param[in] fullModel full model
     * @param[in] traversal traversal on which to run the loop
     * @param[in] jointPos  joint positions for the full model
     * @param[out] traversalBase_H_link  traversalBase_H_link[i] will store the traversalBase_H_i transform
     */
    void computeTransformToTraversalBase(const Model& fullModel,
                                         const Traversal& traversal,
                                         const JointPosDoubleArray& jointPos,
                                               LinkPositions& traversalBase_H_link);

    /**
     * Run the computeTransformToTraversalBase for all the
     * traversal in the subModelDecomposition, and store the
     * results in the linkPos array.
     *
     * @param[in] fullModel full model
     * @param[in] subModelDecomposition model decomposition on which the loop will run
     * @param[in] jointPos  joint positions for the full model
     * @param[out] subModelBase_H_link  subModelBase_H_link[i] will store the subModelBase_H_i transform
     */
    void computeTransformToSubModelBase(const Model& fullModel,
                                        const SubModelDecomposition& subModelDecomposition,
                                        const JointPosDoubleArray& jointPos,
                                              LinkPositions& subModelBase_H_link);

}

#endif
