using Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;
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

                // case "4": Branch & Bound Simplex   -> Member 2, wire up once delivered
                // case "5": Cutting Plane Algorithm  -> Member 2, wire up once delivered
                // case "6": Branch & Bound Knapsack  -> Member 4, wire up once delivered
                // case "7": Sensitivity Analysis     -> Member 3, gate on currentResult.IsOptimal
                // case "8": Duality                  -> Member 3

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
}
