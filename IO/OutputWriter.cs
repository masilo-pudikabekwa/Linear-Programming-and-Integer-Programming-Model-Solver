using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;
using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.IO;

// SolutionResult -> text file. Naming convention for the team: this is the base
// Write<X>Result method (WriteResult, for plain LP/IP results). Member 2 should add
// WriteBranchAndBoundResult(...) and Member 3 WriteSensitivityResult(...) following
// the same Write<X>Result pattern rather than inventing their own logging style.
public static class OutputWriter 
{
    // Spec requirement: All decimal values should be rounded to three points
    private const string DecimalFormat = "F3";

    public static void WriteResult(string filePath, LPModel model, SolutionResult result, string algorithmName)
    {
        var sb = new StringBuilder();

        sb.AppendLine($"Algorithm: {algorithmName}");
        sb.AppendLine();

        // Spec: "The output file should contain the Canonical form..." — the ORIGINAL
        // model as entered, not a relaxed/canonical simplex form. (Naming carried over
        // from the spec's own wording.)
        AppendCanonicalForm(sb, model);
        sb.AppendLine();

        if (!result.IsFeasible)
        {
            sb.AppendLine("The model is infeasible.");
            if (!string.IsNullOrWhiteSpace(result.ErrorMessage))
            {
                sb.AppendLine(result.ErrorMessage);
            }
            File.WriteAllText(filePath, sb.ToString());
            return;
        }

        if (!result.IsBounded)
        {
            sb.AppendLine("The model is unbounded.");
            if (!string.IsNullOrWhiteSpace(result.ErrorMessage))
            {
                sb.AppendLine(result.ErrorMessage);
            }
            File.WriteAllText(filePath, sb.ToString());
            return;
        }

        // Spec: "...and all the table iterations of the algorithm" — dump every stored
        // iteration in order, not just the final tableau.
        for (int i = 0; i < result.Iterations.Count; i++)
        {
            sb.AppendLine($"Iteration {i}:");
            AppendTableau(sb, result.Iterations[i]);
            sb.AppendLine();
        }

        sb.AppendLine("==== Optimal Solution ====");
        sb.AppendLine($"Objective Value: {result.ObjectiveValue.ToString(DecimalFormat, CultureInfo.InvariantCulture)}");
        for (int i = 0; i < result.VariableValues.Length; i++)
        {
            sb.AppendLine(
                $"x{i + 1} = {result.VariableValues[i].ToString(DecimalFormat, CultureInfo.InvariantCulture)}"
            );
        }

        File.WriteAllText(filePath, sb.ToString());
    }

    // Renders the model exactly as entered (objective, constraints, sign restrictions) —
    // shared by WriteResult and available to Member 2/3's own Write<X>Result overloads
    // so every output file starts with the same header regardless of algorithm.
    public static void AppendCanonicalForm(StringBuilder sb, LPModel model)
    {
        sb.Append(model.Objective == ObjectiveType.Max ? "max " : "min ");
        sb.AppendLine(string.Join(" ", model.ObjectiveCoefficients.Select(FormatSigned)));

        foreach (var c in model.Constraints)
        {
            string relation = c.Relation switch
            {
                RelationType.LessThanOrEqualTo => "<=",
                RelationType.GreaterThanOrEqualTo => ">=",
                RelationType.EqualTo => "=",
                _ => "?"
            };
            sb.AppendLine($"{string.Join(" ", c.Coefficients.Select(FormatSigned))} {relation}" +
                          $"{c.RHS.ToString(DecimalFormat, CultureInfo.InvariantCulture)}");

        }

        sb.AppendLine(string.Join(" ", model.Restrictions.Select(FormatRestriction)));
    }

    public static void AppendTableau(StringBuilder sb, Tableau tableau) 
    { 
        int rows = tableau.Matrix.GetLength(0);
        int cols = tableau.Matrix.GetLength(1);

        for (int r = 0; r < rows; r++)
        {
            var rowValues = new string[cols];
            for (int c = 0; c < cols; c++)
            {
                rowValues[c] = tableau.Matrix[r, c].ToString(DecimalFormat, CultureInfo.InvariantCulture);
            }

            // Tag each row with its basic variable where known, so the file is readable
            // without cross-referencing BasicVariableIndices in a separate place.
            string rowLabel = (r > 0 && r - 1 < tableau.BasicVariableIndices.Length)
                ? $"x{tableau.BasicVariableIndices[r - 1] + 1}" : "z";

            sb.AppendLine($"{rowLabel,-4} | {string.Join("  ", rowValues)}");
        }
    }

    private static string FormatSigned(double value)
    {
        string formatted = value.ToString(DecimalFormat, CultureInfo.InvariantCulture);
        return value >= 0 ? $"+{formatted}" : formatted;
    }

    private static string FormatRestriction(SignRestriction r) => r switch
    {
        SignRestriction.Positive => "+",
        SignRestriction.Negative => "-",
        SignRestriction.Unrestricted => "urs",
        SignRestriction.Integer => "int",
        SignRestriction.Binary => "bin",
        _ => "?"
    };

}