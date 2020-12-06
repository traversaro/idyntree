/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OptimizationProblem.h>
#include <iDynTree/Optimizers/IpoptInterface.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <vector>
#include <memory>

class TestProblem : public iDynTree::optimization::OptimizationProblem {
    double m_b1, m_c1;
    double m_plusInfinity, m_minusInfinity;
    iDynTree::VectorDynSize m_variables, m_expectedVariables;

public:
    TestProblem()
    :m_plusInfinity(1e19)
    ,m_minusInfinity(-1e19)
    {}

    virtual ~TestProblem() override {}

    virtual bool prepare() override {
        m_b1 = 1.0;
        m_c1 = -1.0;

        return true;
    }

    virtual void reset() override {}

    virtual unsigned int numberOfVariables() override {
        return 1;
    }

    virtual unsigned int numberOfConstraints() override {
        return 1;
    }

    virtual bool getGuess(iDynTree::VectorDynSize &guess) override {
        guess.resize(1);
        guess.zero();
        guess[0] = 0.0;
        return true;
    }

    virtual bool getConstraintsBounds(iDynTree::VectorDynSize& constraintsLowerBounds, iDynTree::VectorDynSize& constraintsUpperBounds) override {
        constraintsLowerBounds.resize(1);
        constraintsUpperBounds.resize(1);
        constraintsLowerBounds(0) = m_c1;
        //constraintsLowerBounds(1) = m_c2;
        constraintsUpperBounds(0) = m_plusInfinity;
        //constraintsUpperBounds(1) = m_plusInfinity;
        return true;
    }

    virtual bool getVariablesUpperBound(iDynTree::VectorDynSize& variablesUpperBound) override {
        return false;
    } //return false if not upper bounded

    virtual bool getVariablesLowerBound(iDynTree::VectorDynSize& variablesLowerBound) override {
        return false;
    } //return false if not lower bounded

    virtual bool getConstraintsJacobianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns) override {
        nonZeroElementRows.resize(1);
        nonZeroElementColumns.resize(1);

        nonZeroElementRows[0] = 0;
        //nonZeroElementRows[1] = 1;

        nonZeroElementColumns = nonZeroElementRows;

        return true;
    }

    virtual bool getHessianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns) override {
        nonZeroElementRows.resize(1);
        nonZeroElementColumns.resize(1);

        nonZeroElementRows[0] = 0;
        //nonZeroElementRows[1] = 1;

        nonZeroElementColumns = nonZeroElementRows;

        return true;
    } //costs and constraints together

    virtual bool setVariables(const iDynTree::VectorDynSize& variables) override {
        ASSERT_IS_TRUE(variables.size()==1);
        m_variables = variables;
        return true;
    }

    virtual bool evaluateCostFunction(double& costValue) override {
        costValue = m_b1 * m_variables(0);
        return true;
    }

    virtual bool evaluateCostGradient(iDynTree::VectorDynSize& gradient) override {
        gradient.resize(numberOfVariables());
        gradient(0) = m_b1;
        return true;
    }

    virtual bool evaluateCostHessian(iDynTree::MatrixDynSize& hessian) override {
        hessian.resize(1,1);
        hessian.zero();
        hessian(0,0) = 0.0;
        //hessian(1,1) = 2*m_a2;
        return true;
    }

    virtual bool evaluateConstraints(iDynTree::VectorDynSize& constraints) override {
        constraints.resize(1);
        constraints = m_variables;
        return true;
    }

    virtual bool evaluateConstraintsJacobian(iDynTree::MatrixDynSize& jacobian) override {
        jacobian.resize(1,1);
        jacobian(0,0) = 1.0;
        //jacobian(1,1) = 1.0;
        return true;
    } //using dense matrices, but the sparsity pattern is still obtained

    virtual bool evaluateConstraintsHessian(const iDynTree::VectorDynSize& constraintsMultipliers, iDynTree::MatrixDynSize& hessian) override {
        hessian.resize(1,1);
        hessian.zero();
        return true;
    } //using dense matrices, but the sparsity pattern is still obtained

    bool setPlusInfinity(double plusInfinity) {
        if (plusInfinity < 0)
            return false;

        m_plusInfinity = plusInfinity;
        return true;
    }

    bool setMinusInfinity(double minusInfinity) {
        if (minusInfinity > 0)
            return false;

        m_minusInfinity = minusInfinity;
        return true;
    }

    double expectedMinimum() {
        

        return m_b1*m_c1;
    }

    const iDynTree::VectorDynSize& expectedVariables() {
        m_expectedVariables.resize(1);
        m_expectedVariables(0) = m_c1;

        return m_expectedVariables;
    }
};

int main(){
    iDynTree::optimization::IpoptInterface ipoptSolver;
    std::shared_ptr<TestProblem> problem(new TestProblem);
    iDynTree::VectorDynSize dummy1, dummy2, dummy3;

    //ASSERT_IS_TRUE(ipoptSolver.setIpoptOption("nlp_lower_bound_inf", -1.0e20));
    ASSERT_IS_TRUE(ipoptSolver.setIpoptOption("print_level", 12));

    ASSERT_IS_TRUE(problem->setMinusInfinity(ipoptSolver.minusInfinity()));
    ASSERT_IS_TRUE(problem->setPlusInfinity(ipoptSolver.plusInfinity()));
    ASSERT_IS_TRUE(ipoptSolver.setProblem(problem));
    double testTolerance = 5e-5;
    for (int i = 0; i < 5; ++i){
        ASSERT_IS_TRUE(ipoptSolver.solve());
        double optimalCost;
        ASSERT_IS_TRUE(ipoptSolver.getOptimalCost(optimalCost));
        ASSERT_EQUAL_DOUBLE_TOL(optimalCost, problem->expectedMinimum(), testTolerance);
        iDynTree::VectorDynSize solution;
        ASSERT_IS_TRUE(ipoptSolver.getPrimalVariables(solution));
        ASSERT_EQUAL_VECTOR_TOL(solution, problem->expectedVariables(), testTolerance);
        ASSERT_IS_TRUE(ipoptSolver.getDualVariables(dummy1, dummy2, dummy3));
        std::cerr << "solution: " << solution(0) << std::endl;
    }
    return EXIT_SUCCESS;
}
