using System;
using System.Collections.Generic;
using System.Linq;

namespace Algorithms
{
    /// <summary>
    /// Branch-and-Bound wrapper that solves integer versions of an LP by repeatedly
    /// solving LP relaxations (via a provided solver delegate) and branching on the most
    /// fractional variable. Designed to be easy to integrate: remove/replace the
    /// "integration stubs" below with your project's LP model/solver types.
    /// </summary>
    public class BranchAndBoundSimplex
    {
        // --- Integration stubs ------------------------------------------------
        // Remove or replace the types below with your repo's model/solver types.
        // They exist only so this file compiles standalone for easier integration.

        public enum ConstraintSense { LessOrEqual, GreaterOrEqual, Equal }

        public class Constraint
        {
            public double[] Coefficients { get; }
            public ConstraintSense Sense { get; }
            public double RightHandSide { get; }

            public Constraint(double[] coefficients, ConstraintSense sense, double rhs)
            {
                Coefficients = coefficients;
                Sense = sense;
                RightHandSide = rhs;
            }
        }

        public class LPModel
        {
            // number of decision variables (assumes variables are ordered 0..n-1)
            public int NumVariables { get; private set; }

            // objective: maximize c^T x (or minimize if your solver expects that; adapt accordingly)
            public double[] ObjectiveCoefficients { get; private set; }

            // list of constraints (will be cloned)
            public List<Constraint> Constraints { get; } = new List<Constraint>();

            public LPModel(int numVariables, double[] objectiveCoefficients)
            {
                NumVariables = numVariables;
                ObjectiveCoefficients = objectiveCoefficients;
            }

            public LPModel Clone()
            {
                var clone = new LPModel(NumVariables, (double[])ObjectiveCoefficients.Clone());
                clone.Constraints.AddRange(Constraints.Select(c =>
                    new Constraint((double[])c.Coefficients.Clone(), c.Sense, c.RightHandSide)));
                return clone;
            }

            public void AddConstraint(Constraint c) => Constraints.Add(c);
        }

        public class LPSolution
        {
            public bool IsFeasible { get; set; }
            public double ObjectiveValue { get; set; }
            public double[] VariableValues { get; set; } = Array.Empty<double>();
        }

        // Solver delegate: given an LPModel, return LPSolution.
        // The solver must solve the LP relaxation. For maximization problems,
        // ObjectiveValue should be the objective value (higher = better).
        // Replace with the project's existing solver interface when integrating.
        public delegate LPSolution LPSolverDelegate(LPModel model);

        // --- End of integration stubs -----------------------------------------

        private readonly LPSolverDelegate _solver;
        private readonly double _integralityTolerance;
        private readonly int _maxNodes;

        /// <summary>
        /// Create a B&B instance.
        /// - solver: function that solves an LPModel and returns an LPSolution.
        /// - integralityTolerance: how close to an integer a variable must be to count as integer (default 1e-6).
        /// - maxNodes: optional limit on nodes searched (0 or negative => no limit).
        /// </summary>
        public BranchAndBoundSimplex(LPSolverDelegate solver, double integralityTolerance = 1e-6, int maxNodes = 0)
        {
            _solver = solver ?? throw new ArgumentNullException(nameof(solver));
            _integralityTolerance = integralityTolerance;
            _maxNodes = maxNodes;
        }

        /// <summary>
        /// Solve the integer program with branch-and-bound.
        /// - root: the LP relaxation model (without integrality constraints).
        /// - integerVariableIndices: indices of variables that must be integer (if null, all variables are integer).
        /// Returns the best integer-feasible solution found (or null if none).
        /// </summary>
        public LPSolution Solve(LPModel root, IEnumerable<int>? integerVariableIndices = null)
        {
            if (root == null) throw new ArgumentNullException(nameof(root));
            var integerVars = (integerVariableIndices == null)
                ? Enumerable.Range(0, root.NumVariables).ToArray()
                : integerVariableIndices.ToArray();

            LPSolution bestSolution = null;
            double bestObjective = double.NegativeInfinity;

            int nodesVisited = 0;

            // Node structure: model + optional branching info
            var stack = new Stack<LPModel>();
            stack.Push(root.Clone());

            while (stack.Count > 0)
            {
                if (_maxNodes > 0 && nodesVisited >= _maxNodes) break;
                var nodeModel = stack.Pop();
                nodesVisited++;

                // Solve LP relaxation at this node
                var sol = _solver(nodeModel);
                if (sol == null || !sol.IsFeasible)
                {
                    // infeasible => prune
                    continue;
                }

                // bounding: if objective <= bestObjective (for maximization), prune
                if (bestSolution != null && sol.ObjectiveValue <= bestObjective + 1e-12)
                {
                    continue;
                }

                // check integrality
                int fractionalIndex = -1;
                double fractionalAmount = 0.0;
                foreach (var i in integerVars)
                {
                    double val = (i < sol.VariableValues.Length) ? sol.VariableValues[i] : 0.0;
                    double rounded = Math.Round(val);
                    double diff = Math.Abs(val - rounded);
                    if (diff > _integralityTolerance)
                    {
                        // fractional
                        double fracPart = Math.Abs(val - Math.Floor(val));
                        // choose most fractional variable (closest to 0.5 -> most fractional)
                        double closenessToHalf = Math.Abs(fracPart - 0.5);
                        // Convert closeness to a score for selecting variable with smallest closeness
                        if (fractionalIndex == -1 || closenessToHalf < fractionalAmount)
                        {
                            fractionalIndex = i;
                            fractionalAmount = closenessToHalf;
                        }
                    }
                }

                if (fractionalIndex == -1)
                {
                    // All integer variables are integer -> update best
                    bestSolution = sol;
                    bestObjective = sol.ObjectiveValue;
                    continue;
                }

                // Branch on fractionalIndex: create two child nodes
                double xVal = sol.VariableValues[fractionalIndex];
                double floorVal = Math.Floor(xVal);
                double ceilVal = Math.Ceiling(xVal);

                // Left branch: x_i <= floor(xVal)
                var left = nodeModel.Clone();
                var leftCoeffs = new double[left.NumVariables];
                leftCoeffs[fractionalIndex] = 1.0;
                left.AddConstraint(new Constraint(leftCoeffs, ConstraintSense.LessOrEqual, floorVal));
                // Right branch: x_i >= ceil(xVal)
                var right = nodeModel.Clone();
                var rightCoeffs = new double[right.NumVariables];
                rightCoeffs[fractionalIndex] = 1.0;
                right.AddConstraint(new Constraint(rightCoeffs, ConstraintSense.GreaterOrEqual, ceilVal));

                // Heuristic: search the bound that is more promising first.
                // We'll solve both quickly to estimate which to push first; to avoid extra solves,
                // push the branch with larger LP relaxation objective first if possible.
                // For simplicity and to avoid extra solves, push right then left (stack => left solved next).
                // If you want best-first, solve children here and order by objective (extra solves).
                stack.Push(right);
                stack.Push(left);
            }

            return bestSolution;
        }
    }
}
