using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

public class CuttingPlaneAlgorithm : IAlgorithm
{
    private const double Epsilon = 1e-9;
    private const int MaxIterations = 1000;

    public SolutionResult Solve(LPModel model)
    {
        var result = new SolutionResult();

        // Use the existing Primal Simplex to solve the current LP relaxation.
        var simplex = new PrimalSimplex();

        // Work on a copy so that the original model is not changed.
        LPModel currentModel = CloneModel(model);

        for (int iteration = 0; iteration < MaxIterations; iteration++)
        {

            // Solve the current LP relaxation.

            SolutionResult lpResult = simplex.Solve(currentModel);

            if (!lpResult.IsFeasible)
            {
                result.IsFeasible = false;
                result.ErrorMessage =
                    "The LP relaxation is infeasible.";

                return result;
            }

            if (!lpResult.IsBounded)
            {
                result.IsBounded = false;
                result.ErrorMessage =
                    "The LP relaxation is unbounded.";

                return result;
            }

            // Check if the solution is already integer.

            int fractionalVariable =
                FindFractionalVariable(lpResult.VariableValues);

            if (fractionalVariable == -1)
            {
                // Integer feasible solution has been found.
                result.IsFeasible = true;
                result.IsBounded = true;
                result.IsOptimal = true;

                result.ObjectiveValue =
                    lpResult.ObjectiveValue;

                result.VariableValues =
                    (double[])lpResult.VariableValues.Clone();

                result.FinalTableau =
                    lpResult.FinalTableau;

                return result;
            }

            // Cutting plane generation

            result.IsFeasible = false;
            result.ErrorMessage =
                "A fractional solution was found. "
                + "Cut generation must be supplied by the "
                + "responsible cutting-plane component.";

            return result;
        }

        result.IsFeasible = false;
        result.ErrorMessage =
            "Cutting Plane Algorithm exceeded the iteration limit.";

        return result;
    }


    // Finds a variable with a non-integer value.

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

    // Creates a copy of the LP model.
    //
    // This allows the Cutting Plane algorithm to use its own copy without modifying the original model loaded by the user

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

                    Relation =
                        constraint.Relation,

                    RHS =
                        constraint.RHS
                }
            );
        }

        return clone;
    }
}
