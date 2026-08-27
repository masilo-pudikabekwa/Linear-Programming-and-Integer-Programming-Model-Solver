using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

public class KnapsackBranchAndBound : IAlgorithm
{
    private class KnapsackItem
    {
        public int OriginalIndex { get; set; }

        public double Value { get; set; }

        public double Weight { get; set; }

        public double Ratio => Value / Weight;
    }

    private readonly List<KnapsackItem> items = new();

    private double capacity;

    public SolutionResult Solve(LPModel model)
    {
        BuildItems(model);

        double bestValue = 0;
        bool[]? bestSolution = null;

        Stack<KnapsackNode> stack = new();

        var root = new KnapsackNode
        {
            Level = -1,
            Value = 0,
            Weight = 0,
            Included = new bool[items.Count]
        };

        root.Bound = CalculateBound(root);

        stack.Push(root);

        while (stack.Count > 0)
        {
            var current = stack.Pop();

            if (current.Bound <= bestValue)
                continue;

            if (current.Level >= items.Count - 1)
                continue;

            int next = current.Level + 1;

            bool[] includeDecision =
                (bool[])current.Included.Clone();

            includeDecision[next] = true;

            var includeNode = new KnapsackNode
            {
                Level = next,
                Weight = current.Weight + items[next].Weight,
                Value = current.Value + items[next].Value,
                Included = includeDecision
            };

            if (includeNode.Weight <= capacity)
            {
                if (includeNode.Value > bestValue)
                {
                    bestValue = includeNode.Value;
                    bestSolution =
                        (bool[])includeNode.Included.Clone();
                }

                includeNode.Bound =
                    CalculateBound(includeNode);

                if (includeNode.Bound > bestValue)
                    stack.Push(includeNode);
            }

            bool[] excludeDecision =
                (bool[])current.Included.Clone();

            excludeDecision[next] = false;

            var excludeNode = new KnapsackNode
            {
                Level = next,
                Weight = current.Weight,
                Value = current.Value,
                Included = excludeDecision
            };

            excludeNode.Bound =
                CalculateBound(excludeNode);

            if (excludeNode.Bound > bestValue)
                stack.Push(excludeNode);
        }

        double[] values = new double[model.VariableCount];

        if (bestSolution != null)
        {
            for (int i = 0; i < bestSolution.Length; i++)
            {
                if (bestSolution[i])
                {
                    int originalIndex =
                        items[i].OriginalIndex;

                    values[originalIndex] = 1;
                }
            }
        }

        return new SolutionResult
        {
            IsOptimal = true,
            IsFeasible = true,
            IsBounded = true,
            ObjectiveValue = bestValue,
            VariableValues = values
        };
    }

    private void BuildItems(LPModel model)
    {
        items.Clear();

        if (model.Constraints.Count == 0)
            throw new InvalidOperationException(
                "Knapsack requires at least one constraint.");

        capacity = model.Constraints[0].RHS;

        for (int i = 0; i < model.VariableCount; i++)
        {
            items.Add(new KnapsackItem
            {
                OriginalIndex = i,
                Value = model.ObjectiveCoefficients[i],
                Weight = model.Constraints[0].Coefficients[i]
            });
        }

        items.Sort((a, b) =>
            b.Ratio.CompareTo(a.Ratio));
    }

    private double CalculateBound(KnapsackNode node)
    {
        if (node.Weight >= capacity)
            return 0;

        double bound = node.Value;
        double totalWeight = node.Weight;

        int level = node.Level + 1;

        while (level < items.Count &&
               totalWeight + items[level].Weight <= capacity)
        {
            totalWeight += items[level].Weight;
            bound += items[level].Value;

            level++;
        }

        if (level < items.Count)
        {
            double remaining =
                capacity - totalWeight;

            bound += remaining *
                     items[level].Ratio;
        }

        return bound;
    }
}