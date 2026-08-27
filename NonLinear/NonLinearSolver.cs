namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

/* Bonus Non-Linear Solver.


 Uses Golden Section Search to minimize:
    f(x) = x^2, over a bounded interval.


    Golden Section Search repeatedly shrinks the interval containing the minimum.
    Since x^2 is convex, the algorithm converges toward x = 0 where the minimum value is 0.
*/

public class NonLinearSolver
{
    public double Solve(
        double left,
        double right,
        double tolerance = 1e-6)
    {
        double phi = (Math.Sqrt(5.0) - 1.0) / 2.0;

        while ((right - left) > tolerance)
        {
            double c =
                right - phi * (right - left);

            double d =
                left + phi * (right - left);

            if (Function(c) < Function(d))
                right = d;
            else
                left = c;
        }

        return (left + right) / 2.0;
    }

    public double Function(double x)
    {
        return x * x;
    }
}