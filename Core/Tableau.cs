namespace Linear_Programming_and_Integer_Programming_Model_Solver.Core;

public class  Tableau
{
    public double[,] Matrix { get; set; }
    public int[] BasicVariableIndices { get; set; }

    public double[,] BasisInverse { get; set; }

    public double[] CB { get; set; }

    public int IterationNumber { get; set; }

    public Tableau()
    {
        Matrix = new double[0, 0];
        BasicVariableIndices = Array.Empty<int>();
        BasisInverse = new double[0, 0];
        CB = Array.Empty<double>();
    }

    public Tableau Clone()
    {
        int rows = Matrix.GetLength(0);
        int cols = Matrix.GetLength(1);

        var matrixCopy = new double[rows, cols];
        Array.Copy(Matrix, matrixCopy, Matrix.Length);

        return new Tableau
        {
            Matrix = matrixCopy,
            BasicVariableIndices = (int[])BasicVariableIndices.Clone(),
            BasisInverse = (double[,])BasisInverse.Clone(),
            CB = (double[])CB.Clone(),
            IterationNumber = IterationNumber
        };
    }

}
