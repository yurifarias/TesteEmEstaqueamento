import Jama.Matrix;

public class SimetriaPorDoisPlanos extends AnalisesPreliminares {

    private Matrix tensorTransformacao;

    private Matrix matrizComponentesEstacasTransformados;
    private Matrix matrizRigidezTransformada;
    private Matrix matrizEsforcosTransformados;
    private Matrix matrizMovElasticoTransformado;

    private double xoYCL;
    private double xoZBL;

    /* Método para calcular os esforços normais de reação nas estacas: [N] = [v']transposta * [p'] */
    public Matrix calcularEsforcosNormais() {

        acharNovasCoordenadas();
        tensorTransformacao = montarTensorTransformacao();

        matrizComponentesEstacasTransformados = calcularComponentesEstacasTransformados();
        matrizRigidezTransformada = calcularRigidezTransformada();
        matrizEsforcosTransformados = calcularEsforcosTransformados();
        matrizMovElasticoTransformado = calcularMovElasticoTransformado();

        return (matrizComponentesEstacasTransformados.transpose()).times(matrizMovElasticoTransformado);
    }

    /* Método para achar as novas posições do eixo x, em relação ao antigo, após a translação dos antigos
     eixos coordenados como aritifício para solucionar casos de estaqueamento simétrico por um plano. */
    protected void acharNovasCoordenadas() {

        double[][] mS = getMatrizRigidez().getArray();

        xoYCL = mS[1][5] / mS[1][1];
        xoZBL = -mS[2][4] / mS[2][2];
    }

    /* Método para criar a matriz de trnasformação [T], ou tensor de transformação [T], referente aos
    casos de estaquemaneto plano em XY ou XZ retornando uma matriz 6 x 6 por apresentar degeneração */
    protected Matrix montarTensorTransformacao() {

        double[][] tensorT = new double[6][6];

        tensorT[0][0] = 1;
        tensorT[0][1] = 0;
        tensorT[0][2] = 0;
        tensorT[0][3] = 0;
        tensorT[0][4] = 0;
        tensorT[0][5] = 0;
        tensorT[1][0] = 0;
        tensorT[1][1] = 1;
        tensorT[1][2] = 0;
        tensorT[1][3] = 0;
        tensorT[1][4] = 0;
        tensorT[1][5] = 0;
        tensorT[2][0] = 0;
        tensorT[2][1] = 0;
        tensorT[2][2] = 1;
        tensorT[2][3] = 0;
        tensorT[2][4] = 0;
        tensorT[2][5] = 0;
        tensorT[3][0] = 0;
        tensorT[3][1] = 0;
        tensorT[3][2] = 0;
        tensorT[3][3] = 1;
        tensorT[3][4] = 0;
        tensorT[3][5] = 0;
        tensorT[4][0] = 0;
        tensorT[4][1] = 0;
        tensorT[4][2] = xoZBL;
        tensorT[4][3] = 0;
        tensorT[4][4] = 1;
        tensorT[4][5] = 0;
        tensorT[5][0] = 0;
        tensorT[5][1] = -xoYCL;
        tensorT[5][2] = 0;
        tensorT[5][3] = 0;
        tensorT[5][4] = 0;
        tensorT[5][5] = 1;

        return new Matrix(tensorT);
    }

    /* Método para calcular a matriz de componentes de estaca transformados [p'], aplicando o tensor de
    transformação [T], retornando uma matriz 6 x n: [p'] = [T] * [p], por apresentar degeneração,
    onde n é número de estacas */
    protected Matrix calcularComponentesEstacasTransformados() {

        return tensorTransformacao.times(getMatrizComponentesEstacas());
    }

    /* Método para calcular a matriz de rigidez transformada [S'], aplicando o tensor de transformação [T],
    retornando uma matriz 6 x 6, por apresentar degeneração: [S'] = [T] * [S] * [T]transposta */
    protected Matrix calcularRigidezTransformada() {

        return tensorTransformacao.times(getMatrizRigidez().times(tensorTransformacao.transpose()));
    }

    /* Método para calcular a matriz de esforços transformados [F'], aplicando o tensor de transformação [T],
    retornando uma matraiz 6 x 1, por apresentar degeneração: [F'] = [T] * [F] */
    protected Matrix calcularEsforcosTransformados() {

        return tensorTransformacao.times(new Matrix(MainActivity.esforcos));
    }

    /* Método para calcular a matriz do movimento elástico transformado do bloco [v'], após
    a transformação da matriz de rigidez [S'] e da matriz do esforços externos [F'],
    retornando uma matriz 6 x 1, por apresentar degeneração: [v'] = [S' ^ -1] * [F'] */
    protected Matrix calcularMovElasticoTransformado() {

        double[][] vL = new double[6][1];

        if (matrizRigidezTransformada.getArray()[0][0] == 0) {

            vL[0][0] = 0;
        } else {

            vL[0][0] = matrizEsforcosTransformados.getArray()[0][0] / matrizRigidezTransformada.getArray()[0][0];
        }

        if (matrizRigidezTransformada.getArray()[1][1] == 0) {

            vL[1][0] = 0;
        } else {

            vL[1][0] = matrizEsforcosTransformados.getArray()[1][0] / matrizRigidezTransformada.getArray()[1][1];
        }

        if (matrizRigidezTransformada.getArray()[2][2] == 0) {

            vL[0][0] = 0;
        } else {

            vL[0][0] = matrizEsforcosTransformados.getArray()[2][0] / matrizRigidezTransformada.getArray()[2][2];
        }

        if (matrizRigidezTransformada.getArray()[3][3] == 0) {

            vL[0][0] = 0;
        } else {

            vL[0][0] = matrizEsforcosTransformados.getArray()[3][0] / matrizRigidezTransformada.getArray()[3][3];
        }

        if (matrizRigidezTransformada.getArray()[4][4] == 0) {

            vL[0][0] = 0;
        } else {

            vL[0][0] = matrizEsforcosTransformados.getArray()[4][0] / matrizRigidezTransformada.getArray()[4][4];
        }

        if (matrizRigidezTransformada.getArray()[5][5] == 0) {

            vL[0][0] = 0;
        } else {

            vL[0][0] = matrizEsforcosTransformados.getArray()[5][0] / matrizRigidezTransformada.getArray()[5][5];
        }

        return new Matrix(vL);
    }
}