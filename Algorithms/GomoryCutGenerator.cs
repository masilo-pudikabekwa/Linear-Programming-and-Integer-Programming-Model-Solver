using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

public static class GomoryCut
{
    private const double Epsilon = 1e-9;

    // Generates a Gomory fractional cut from the final Simplex
    // tableau.

    // The Cutting Plane Algorithm calls this method whenever the
    // current LP relaxation has a fractional solution.

    // The method:

    // 1. Finds a row with a fractional RHS.
    // 2. Calculates the fractional parts of the row coefficients.
    // 3. Builds the Gomory constraint.
    // 4. Returns the constraint to CuttingPlaneAlgorithm.

    public static bool TryGenerateCut(
        SolutionResult solution,
        LPModel model,
        out Constraint cut)
    {
        cut = null!;

        if (solution.FinalTableau == null)
        {
            return false;
        }

        Tableau tableau = solution.FinalTableau;

        int rows = tableau.Matrix.GetLength(0);
        int cols = tableau.Matrix.GetLength(1);

        int rhsColumn = cols - 1;

        // Find a fractional basic row.
        // We only consider rows belonging to the original decision
        // variables. A Gomory fractional cut is generated from a row
        // whose basic integer variable has a fractional RHS.

        int selectedRow = -1;

        for (int r = 1; r < rows; r++)
        {
            double fractionalrhs = tableau.Matrix[r, rhsColumn];

            if (!IsFractional(fractionalrhs))
            {
                continue;
            }

            // Basic variable corresponding to this row.
            int basicVariable =
                tableau.BasicVariableIndices[r - 1];

            // Only generate the standard Gomory cut from a row whosebasic variable is an original decision variable.

            if (basicVariable < model.VariableCount)
            {
                selectedRow = r;
                break;
            }
        }

        if (selectedRow == -1)
        {
            return false;
        }

        // Build the coefficients of the Gomory cut.
        //
        // frac(a1)x1 + frac(a2)x2 + ... >= frac(b)
        //
        // Only the original decision variables are placed in thereturned LP constraint.

        double[] coefficients =
            new double[model.VariableCount];

        for (int j = 0; j < model.VariableCount; j++)
        {
            double value =
                tableau.Matrix[selectedRow, j];

            coefficients[j] =
                FractionalPart(value);
        }

        // Fractional RHS.

        double rhs =
            FractionalPart(
                tableau.Matrix[selectedRow, rhsColumn]
            );

        // Check the generated cut.

        bool hasCoefficient =
            coefficients.Any(
                value => Math.Abs(value) > Epsilon
            );

        if (!hasCoefficient || rhs <= Epsilon)
        {
            return false;
        }

        // Create the actual LP constraint.
        // frac(a1)x1 + frac(a2)x2 + ... >= frac(b)

        //cut = new Constraint
        //{
        //    Coefficients = coefficients,
        //    Relation = RelationType.GreaterThanOrEqualTo,
        //    RHS = rhs
        //};

        return true;
    }

    // Determines whether a value is fractional.


    private static bool IsFractional(double value)
    {
        double rounded =
            Math.Round(value);

        return Math.Abs(value - rounded) > Epsilon;
    }


    // Returns the fractional part of a number.
    //
    // Examples:
    //
    // 4.25 -> 0.25
    // 7.50 -> 0.50

    private static double FractionalPart(double value)
    {
        double floor =
            Math.Floor(value);

        return value - floor;
    }
}
