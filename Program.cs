using Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;
using Linear_Programming_and_Integer_Programming_Model_Solver.Analysis;
using Linear_Programming_and_Integer_Programming_Model_Solver.Core;
using Linear_Programming_and_Integer_Programming_Model_Solver.IO;

internal class Program
{
    // Entry point + menu state machine. Kept thin on purpose: all real logic lives in
    // IAlgorithm implementations, so Members 2-4 only need to add one more case here
    // once their class exists, not touch this file's internals.

    private static LPModel? currentModel = null;
    private static SolutionResult? currentResult = null;

    private static void Main(string[] args)
    {

        bool running = true;
        while (running)
        {

            Console.WriteLine("=== LPR Linear and Integer Programing Solver ===");
            PrintMenu();
            string? choice = Console.ReadLine();

            switch (choice)
            {
                case "1":
                    LoadModel();
                    break;
                case "2":
                    RunAlgorithm(new PrimalSimplex(), "Primal Simplex");
                    break;
                case "3":
                    RunAlgorithm(new RevisedPrimalSimplex(), "Revised Primal Simplex");
                    break;

                case "4":
                    RunAlgorithm(new BranchAndBoundSimplex(), "Branch & Bound Simplex");
                    break;

                case "5":
                    //RunAlgorithm( new CuttingPlaneAlgorithm(),"Cutting Plane Algorithm");
                    break;

                // case "6": Branch & Bound Knapsack  -> Member 4, wire up once delivered

                case "7":
                    RunSensitivityAnalysis();
                    break;

                case "8":
                    RunDuality();
                    break;

                case "0":
                    running = false;
                    break;
                default:
                    Console.Clear();
                    Console.WriteLine("Invalid choice. Please try again.");
                    break;
            }
        }
    }

    //Start up Menu======================================================
    private static void PrintMenu()
    {
        Console.WriteLine();
        Console.WriteLine("1. Load input file");
        Console.WriteLine("2. Solve - Primal Simplex");
        Console.WriteLine("3. Solve - Revised Primal Simplex");
        Console.WriteLine("4. Solve - Branch & Bound Simplex");
        Console.WriteLine("5. Solve - Cutting Plane Algorithm");
        Console.WriteLine("7. Sensitivity Analysis (requires an optimal solve)");
        Console.WriteLine("8. Duality (convert / solve dual / verify)");
        Console.WriteLine("0. Exit");
        Console.Write("Enter Choice > ");
    }

    private static void LoadModel()
    {
        Console.Write("Input file path: ");
        string? path = Console.ReadLine();
        if (string.IsNullOrWhiteSpace(path))
        {
            Console.WriteLine("No path entered.");
            return;
        }

        try
        {
            currentModel = InputParser.ParseFile(path);
            currentResult = null;
            Console.WriteLine($"Loaded model: {currentModel.VariableCount} variables, {currentModel.Constraints.Count} constraints");
        }
        catch (Exception ex) when (ex is FileNotFoundException or FormatException)
        {
            // Parsing errors are expected user-input mistakes, not bugs — report and let
            // the menu loop continue instead of crashing the console app.
            Console.WriteLine($"Failed to load model: {ex.Message}");
        }
    }

    private static void RunAlgorithm(IAlgorithm algorithm, string name)
    {
        var model = currentModel; // local copy so nullability narrows cleanly below
        if (model == null)
        {
            Console.WriteLine("Load a model first (option 1)");
            return;
        }

        var result = algorithm.Solve(model);
        currentResult = result;

        if (!result.IsFeasible)
        {
            Console.WriteLine("Model is infeasible!");
        }
        else if (!result.IsBounded)
        {
            Console.WriteLine("Model is unbounded!");
        }
        else
        {
            Console.WriteLine($"Optimal objective value: {result.ObjectiveValue:F3}");
            for (int i = 0; i < result.VariableValues.Length; i++)
                Console.WriteLine($"  x{i + 1} = {result.VariableValues[i]:F3}");
        }

        // Output goes into an "Output" folder next to the executable (bin/...), not
        // wherever the console happened to be launched from. One file per algorithm,
        // named after it — re-running a different algorithm never overwrites another's
        // results, and there's nothing to type or mistype at the console.
        string outputDirectory = Path.Combine(AppContext.BaseDirectory, "Output");
        Directory.CreateDirectory(outputDirectory);
        string outputFilePath = Path.Combine(outputDirectory, $"output_{algorithm.GetType().Name}.txt");
        OutputWriter.WriteResult(outputFilePath, model, result, name);
        Console.WriteLine($"Results written to {outputFilePath}");
    }

    // ------------------------------------------------------------------
    // 7. Sensitivity Analysis — Member 3
    // ------------------------------------------------------------------

    private static void RunSensitivityAnalysis()
    {
        // Gate per the spec: this menu only makes sense once SOME algorithm has
        // produced an optimal solve — doesn't matter which one (Primal, Revised,
        // B&B), since SensitivityAnalyzer only reads FinalTableau/CB/BasisInverse.
        if (currentModel == null || currentResult == null || !currentResult.IsOptimal)
        {
            Console.WriteLine("Solve a model to optimality first (options 2-4).");
            return;
        }

        var analyzer = new SensitivityAnalyzer(currentResult.FinalTableau, currentModel);
        bool inSubmenu = true;

        while (inSubmenu)
        {
            Console.WriteLine();
            Console.WriteLine("--- Sensitivity Analysis ---");
            Console.WriteLine("1. Range non-basic variable (objective coefficient)");
            Console.WriteLine("2. Apply change to non-basic variable");
            Console.WriteLine("3. Range basic variable (objective coefficient)");
            Console.WriteLine("4. Apply change to basic variable");
            Console.WriteLine("5. Range RHS");
            Console.WriteLine("6. Apply change to RHS");
            Console.WriteLine("7. Range non-basic column (scale factor)");
            Console.WriteLine("8. Apply change to non-basic column");
            Console.WriteLine("9. Add new activity");
            Console.WriteLine("10. Add new constraint");
            Console.WriteLine("11. Shadow prices");
            Console.WriteLine("0. Back to main menu");
            Console.Write("Enter Choice > ");
            string? choice = Console.ReadLine();

            try
            {
                switch (choice)
                {
                    case "1":
                        {
                            int j = ReadInt("Variable/column index: ");
                            var (lo, hi) = analyzer.RangeNonBasicVariable(j);
                            Console.WriteLine($"c_{j} can range in [{lo}, {hi}] while the basis stays optimal.");
                            break;
                        }
                    case "2":
                        {
                            int j = ReadInt("Variable/column index: ");
                            double v = ReadDouble("New objective coefficient value: ");
                            var result = analyzer.ApplyChangeNonBasicVariable(j, v);
                            PrintSolutionResult(result);
                            break;
                        }
                    case "3":
                        {
                            int j = ReadInt("Variable/column index: ");
                            var (lo, hi) = analyzer.RangeBasicVariable(j);
                            Console.WriteLine($"c_{j} can range in [{lo}, {hi}] while the basis stays optimal.");
                            break;
                        }
                    case "4":
                        {
                            int j = ReadInt("Variable/column index: ");
                            double v = ReadDouble("New objective coefficient value: ");
                            var result = analyzer.ApplyChangeBasicVariable(j, v);
                            PrintSolutionResult(result);
                            break;
                        }
                    case "5":
                        {
                            int i = ReadInt("Constraint index: ");
                            var (lo, hi) = analyzer.RangeRHS(i);
                            Console.WriteLine($"b_{i} can range in [{lo}, {hi}] while the basis stays feasible.");
                            break;
                        }
                    case "6":
                        {
                            int i = ReadInt("Constraint index: ");
                            double v = ReadDouble("New RHS value: ");
                            var result = analyzer.ApplyChangeRHS(i, v);
                            PrintSolutionResult(result);
                            break;
                        }
                    case "7":
                        {
                            int j = ReadInt("Variable/column index: ");
                            var (lo, hi) = analyzer.RangeNonBasicColumn(j);
                            Console.WriteLine($"Column scale factor t can range in [{lo}, {hi}] while the basis stays optimal.");
                            break;
                        }
                    case "8":
                        {
                            int j = ReadInt("Variable/column index: ");
                            var newColumn = ReadVector("New column values (space-separated, one per constraint): ", currentModel.Constraints.Count);
                            var result = analyzer.ApplyChangeNonBasicColumn(j, newColumn);
                            PrintSolutionResult(result);
                            break;
                        }
                    case "9":
                        {
                            double coeff = ReadDouble("New activity's objective coefficient: ");
                            var column = ReadVector("New activity's constraint column (space-separated, one per constraint): ", currentModel.Constraints.Count);
                            var result = analyzer.AddNewActivity(coeff, column);
                            PrintSolutionResult(result);
                            break;
                        }
                    case "10":
                        {
                            var coeffs = ReadVector("New constraint coefficients (space-separated, one per decision variable): ", currentModel.VariableCount);
                            var relation = ReadRelation();
                            double rhs = ReadDouble("New constraint RHS: ");
                            var result = analyzer.AddNewConstraint(new Constraint(coeffs, relation, rhs));
                            PrintSolutionResult(result);
                            break;
                        }
                    case "11":
                        {
                            var prices = analyzer.ShadowPrices();
                            for (int i = 0; i < prices.Length; i++)
                                Console.WriteLine($"  y{i + 1} = {prices[i]:F4}");
                            break;
                        }
                    case "0":
                        inSubmenu = false;
                        break;
                    default:
                        Console.WriteLine("Invalid choice. Please try again.");
                        break;
                }
            }
            catch (Exception ex)
            {
                // Bad index, wrong variable state (basic vs non-basic), etc. — report
                // and stay in the submenu instead of dropping back to the main menu.
                Console.WriteLine($"Error: {ex.Message}");
            }
        }
    }

    // ------------------------------------------------------------------
    // 8. Duality — Member 3
    // ------------------------------------------------------------------

    private static void RunDuality()
    {
        if (currentModel == null)
        {
            Console.WriteLine("Load a model first (option 1)");
            return;
        }

        var converter = new DualityConverter();
        var dualModel = converter.ToDual(currentModel, out var labels);

        Console.WriteLine();
        Console.WriteLine("--- Dual Model ---");
        Console.WriteLine($"Objective: {dualModel.Objective}");
        Console.WriteLine($"Dual variables: {string.Join(", ", labels)}");
        Console.WriteLine($"Objective coefficients: [{string.Join(", ", dualModel.ObjectiveCoefficients)}]");
        for (int i = 0; i < dualModel.Constraints.Count; i++)
        {
            var c = dualModel.Constraints[i];
            Console.WriteLine($"  [{string.Join(", ", c.Coefficients)}] {RelationSymbol(c.Relation)} {c.RHS}");
        }

        Console.Write("Solve the dual and verify against the current primal result? (y/n): ");
        if (Console.ReadLine()?.Trim().ToLower() != "y") return;

        var dualResult = new PrimalSimplex().Solve(dualModel);
        Console.WriteLine();
        Console.WriteLine("--- Dual Solve Result ---");
        PrintSolutionResult(dualResult);

        if (currentResult == null)
        {
            Console.WriteLine("No primal result to compare against yet — solve the primal first (options 2-4).");
            return;
        }

        var verifier = new DualityVerifier();
        var (isStrong, explanation) = verifier.Verify(currentResult, dualResult);
        Console.WriteLine();
        Console.WriteLine($"--- Duality Verification ({(isStrong ? "STRONG" : "NOT STRONG")}) ---");
        Console.WriteLine(explanation);
    }

    // ------------------------------------------------------------------
    // Small console I/O helpers shared by the submenus above
    // ------------------------------------------------------------------

    private static int ReadInt(string prompt)
    {
        Console.Write(prompt);
        return int.Parse(Console.ReadLine()!.Trim());
    }

    private static double ReadDouble(string prompt)
    {
        Console.Write(prompt);
        return double.Parse(Console.ReadLine()!.Trim());
    }

    private static double[] ReadVector(string prompt, int expectedLength)
    {
        Console.Write(prompt);
        var values = Console.ReadLine()!
            .Trim()
            .Split(' ', StringSplitOptions.RemoveEmptyEntries)
            .Select(double.Parse)
            .ToArray();

        if (values.Length != expectedLength)
            throw new FormatException($"Expected {expectedLength} values, got {values.Length}.");

        return values;
    }

    private static RelationType ReadRelation()
    {
        Console.Write("Relation (<=, >=, =): ");
        return Console.ReadLine()!.Trim() switch
        {
            "<=" => RelationType.LessThanOrEqualTo,
            ">=" => RelationType.GreaterThanOrEqualTo,
            "=" => RelationType.EqualTo,
            var other => throw new FormatException($"Unrecognized relation '{other}'.")
        };
    }

    private static string RelationSymbol(RelationType relation) => relation switch
    {
        RelationType.LessThanOrEqualTo => "<=",
        RelationType.GreaterThanOrEqualTo => ">=",
        RelationType.EqualTo => "=",
        _ => "?"
    };

    private static void PrintSolutionResult(SolutionResult result)
    {
        if (!result.IsFeasible)
        {
            Console.WriteLine("Infeasible.");
        }
        else if (!result.IsBounded)
        {
            Console.WriteLine("Unbounded.");
        }
        else
        {
            Console.WriteLine($"Objective: {result.ObjectiveValue:F4}");
            for (int i = 0; i < result.VariableValues.Length; i++)
                Console.WriteLine($"  x{i + 1} = {result.VariableValues[i]:F4}");
        }

        if (!string.IsNullOrEmpty(result.ErrorMessage))
            Console.WriteLine(result.ErrorMessage);
    }
}