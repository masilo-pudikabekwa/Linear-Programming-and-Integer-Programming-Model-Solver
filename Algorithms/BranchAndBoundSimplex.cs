using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

public class BranchAndBoundSimplex : IAlgorithm
{
    private const double Epsilon = 1e-9;
    private const int MaxNodes = 1000;

    public SolutionResult Solve(LPModel model)
    {
        var result = new SolutionResult();

        // Branch-and-Bound assumes an integer programming problem.
        // The Primal Simplex solver will be used to solve every LP relaxation.

        var simplex = new PrimalSimplex();

        SolutionResult? bestSolution = null;
        double bestObjective = double.NegativeInfinity;

        var nodes = new Stack<LPModel>();
        nodes.Push(CloneModel(model));

        int nodesVisited = 0;

        while (nodes.Count > 0 && nodesVisited < MaxNodes)
        {
            nodesVisited++;

            LPModel currentModel = nodes.Pop();

          
            // STEP 1: Solve the LP relaxation using Primal Simplex
           
            SolutionResult relaxation = simplex.Solve(currentModel);

            // If LP relaxation is infeasible, prune this node.
            if (!relaxation.IsFeasible)
            {
                continue;
            }

            // If LP relaxation is unbounded, report it.
            if (!relaxation.IsBounded)
            {
                result.IsBounded = false;
                result.ErrorMessage = "LP relaxation is unbounded.";
                return result;
            }

          
            // STEP 2: Bounding

            if (bestSolution != null &&
                relaxation.ObjectiveValue <= bestObjective + Epsilon)
            {
                // This node cannot improve the current best solution.
                continue;
            }

           
            // STEP 3: Check whether all variables are integer

            int branchingVariable = FindFractionalVariable(
                relaxation.VariableValues
            );

            // No fractional variable means we found an integer solution.
            if (branchingVariable == -1)
            {
                bestSolution = relaxation;
                bestObjective = relaxation.ObjectiveValue;
                continue;
            }

        
            // STEP 4: Branch

            double value = relaxation.VariableValues[branchingVariable];

            double floorValue = Math.Floor(value);
            double ceilValue = Math.Ceiling(value);

     
            // LEFT BRANCH:
            // x_i <= floor(x_i)

            LPModel leftModel = CloneModel(model: currentModel);

            var leftCoefficients =
                new double[currentModel.VariableCount];

            leftCoefficients[branchingVariable] = 1.0;

            leftModel.Constraints.Add(
                new Constraint(
                    leftCoefficients,
                    RelationType.LessThanOrEqualTo,
                    floorValue
                )
            );


            // RIGHT BRANCH:
            // x_i >= ceil(x_i)

            LPModel rightModel = CloneModel(model: currentModel);

            var rightCoefficients =
                new double[currentModel.VariableCount];

            rightCoefficients[branchingVariable] = 1.0;

            rightModel.Constraints.Add(
                new Constraint(
                    rightCoefficients,
                    RelationType.GreaterThanOrEqualTo,
                    ceilValue
                )
            );

            // STEP 5: Put child nodes onto stack

            nodes.Push(rightModel);
            nodes.Push(leftModel);
        }

        // STEP 6: Return best integer solution

        if (bestSolution == null)
        {
            result.IsFeasible = false;
            result.ErrorMessage =
                "No integer feasible solution was found.";
            return result;
        }

        result.IsFeasible = true;
        result.IsBounded = true;
        result.IsOptimal = true;

        result.ObjectiveValue = bestSolution.ObjectiveValue;
        result.VariableValues =
            (double[])bestSolution.VariableValues.Clone();

        return result;
    }


    // Finds the first variable that is not an integer.

    private static int FindFractionalVariable(double[] values)
    {
        for (int i = 0; i < values.Length; i++)
        {
            double rounded = Math.Round(values[i]);

            if (Math.Abs(values[i] - rounded) > Epsilon)
            {
                return i;
            }
        }

        return -1;
    }

    // Creates a copy of the LP model so that branching does not modify
    // the parent node.
    
    private static LPModel CloneModel(LPModel model)
    {
        var clone = new LPModel
        {
            VariableCount = model.VariableCount,
            Objective = model.Objective,
            ObjectiveCoefficients =
                (double[])model.ObjectiveCoefficients.Clone()
        };

        foreach (var constraint in model.Constraints)
        {
            clone.Constraints.Add(
                new Constraint
                {
                    Coefficients =
                        (double[])constraint.Coefficients.Clone(),

                    Relation = constraint.Relation,

                    RHS = constraint.RHS
                }
            );
        }

        return clone;
    }
}
