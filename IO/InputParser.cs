using System.Globalization;
using System.Text.RegularExpressions;
using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.IO;

//Text file -> LPModel. This is the only class that should touch the raw input
// file format — everything downstream works with LPModel and never sees a string again.

public static class InputParser
{
    // GOTCHA #2: the relation operator and RHS are concatenated with no space,
    // e.g. "+11 +8 +6 +14 +10 +10 <=40" -> last token is "<=40"

    private static readonly Regex RelationRhsPattern = new(@"^(<=|>=|=)(-?\d+\.?\d*)$", RegexOptions.Compiled);

    public static LPModel ParseFile(string filePath)
    {
        if (!File.Exists(filePath))
        {
            throw new FileNotFoundException($"Input file not found: {filePath}");
        }

        var lines = File.ReadAllLines(filePath)
        .Where(line => !string.IsNullOrWhiteSpace(line))
        .ToArray();

        return ParseLines(lines);
    }

    public static LPModel ParseLines(string[] lines)
    {
        if (lines.Length < 3)
        {
            throw new FormatException("Input file must contain at least an objective line, one constraint line, and a sign restriction line.");
        }

        var model = new LPModel();

        // Line 1: objective e.g. "max +2 +3 +3 +5 +2 +4"
        var objectiveTokens = lines[0].Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (objectiveTokens.Length < 2)
        {
            throw new FormatException("Objective line must contain 'max'/'min followed by at least one coefficient.");
        }

        model.Objective = objectiveTokens[0].ToLowerInvariant() switch
        {
            "max" => ObjectiveType.Max,
            "min" => ObjectiveType.Min,
            _ => throw new FormatException($"Objective line must start with 'max' or 'min', found '{objectiveTokens[0]}'.")
        };

        // Sign and coefficient are ONE token (e.g. "+2"), not two.
        // double.TryParse with NumberStyles.Float (which includes AllowLeadingSign)
        // handles this natively — no separate sign-token parser needed.
        model.ObjectiveCoefficients = objectiveTokens
            .Skip(1).Select(t => ParseSignedNumber(t, "objective coefficient"))
            .ToArray();

        int varCount = model.ObjectiveCoefficients.Length;

        // Middle lines: one constraint each
        // Everything between line 1 and the last line is a constraint row.
        model.Constraints = new List<Constraint>();
        for (int i = 1; i < lines.Length - 1; i++)
        {
            model.Constraints.Add(ParseConstraintLine(lines[i], varCount, i + 1));
        }

        // Last line: sign restrictions, one token per variable"
        model.Restrictions = ParseRestrictionLine(lines[^1], varCount);

        return model;
    }

    private static Constraint ParseConstraintLine(string line, int expectedVarCount, int lineNumber)
    {
        var tokens = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (tokens.Length < expectedVarCount + 1)
        {
            throw new FormatException($"Line {lineNumber}: expected {expectedVarCount} coefficients plus a relation+RHS token, got {tokens.Length} tokens.");
        }

        string lastToken = tokens[^1];
        var match = RelationRhsPattern.Match(lastToken);
        if (!match.Success)
        {
            throw new FormatException($"Line {lineNumber}: could not parse relation/RHS token '{lastToken}'. Expected a format like '<=40'.");
        }

        var relation = match.Groups[1].Value switch
        {
            "<=" => RelationType.LessThanOrEqualTo,
            ">=" => RelationType.GreaterThanOrEqualTo,
            "=" => RelationType.EqualTo,
            _ => throw new FormatException($"Line {lineNumber}: unknown relation operator '{match.Groups[1].Value}'.")
        };
        double rhs = double.Parse(match.Groups[2].Value, CultureInfo.InvariantCulture);

        // Everything before the relation+RHS token is a signed coefficient, one per variable.
        var coefficientTokens = tokens[..^1];
        if (coefficientTokens.Length != expectedVarCount)
        {
            throw new FormatException($"Line {lineNumber}: expected {expectedVarCount} coefficients, found {coefficientTokens.Length}.");
        }

        var coefficients = coefficientTokens
        .Select(t => ParseSignedNumber(t, $"constraint coefficient on line {lineNumber}"))
        .ToArray();

        return new Constraint(coefficients, relation, rhs);
    }

    private static SignRestriction[] ParseRestrictionLine(string line, int expectedVarCount)
    {
        var tokens = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (tokens.Length != expectedVarCount)
        {
            throw new FormatException($"Sign restriction line: expected {expectedVarCount} tokens (one per variable), found {tokens.Length}.");
        }

        return tokens.Select((t, idx) => t.ToLowerInvariant() switch 
        {
            "+" => SignRestriction.Positive,
            "-" => SignRestriction.Negative,
            "urs" => SignRestriction.Unrestricted,
            "int" => SignRestriction.Integer,
            "bin" => SignRestriction.Binary,
            _ => throw new FormatException($"Unknown sign restriction token '{t}' at position {idx + 1}.")
        }).ToArray();
    }

    private static double ParseSignedNumber(string token, string context)
    {
        // NumberStyles.Float = AllowLeadingWhite | AllowTrailingWhite | AllowLeadingSign
        // | AllowDecimalPoint | AllowExponent — exactly what "+2", "-3.5" need, nothing more.
        if (!double.TryParse(token, NumberStyles.Float, CultureInfo.InvariantCulture, out double value))
        {
            throw new FormatException($"Could not parse {context} token '{token}' as a signed number.");
        }
        return value;
    }
}
