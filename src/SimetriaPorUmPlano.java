import Jama.Matrix;

public class SimetriaPorUmPlano extends AnalisesPreliminares {

    private Matrix matrizComponentesEstacasSubdividida1;
    private Matrix matrizComponentesEstacasSubdividida2;
    private Matrix matrizRigidezSubdividida1;
    private Matrix matrizRigidezSubdividida2;
    private Matrix matrizEsforcosExternosSubdividida1;
    private Matrix matrizEsforcosExternosSubdividida2;

    private Matrix tensorTransformacao1;
    private Matrix tensorTransformacao2;

    private Matrix matrizComponentesEstacasSubTransformada1;
    private Matrix matrizComponentesEstacasSubTransformada2;
    private Matrix matrizRigidezSubTransformada1;
    private Matrix matrizRigidezSubTransformada2;
    private Matrix matrizEsforcosSubTransformada1;
    private Matrix matrizEsforcosSubTransformada2;
    private Matrix matrizMovElasticoSubTransformado1;
    private Matrix matrizMovElasticoSubTransformado2;

    private double fi;
    private double xo;
    private double yo;
    private double zo;

    private double psi;
    private double xoL;
    private double yoL;
    private double zoL;

    /* Método para calcular os esforços normais de reação nas estacas: [N] = [v']transposta * [p'] */
    public Matrix calcularEsforcosNormais(char caso) {

        subdividirMatrizCompontesEstacas(caso);
        subdividirMatrizRigidez(caso);
        subdividirMatrizEsforcos(caso);

        acharNovasCoordenadas(caso);
        montarTensorTransformacao(caso);

        calcularComponentesEstacasTransformados();
        calcularRigidezTransformada();
        calcularEsforcosTransformados();
        calcularMovElasticoTransformado();

        calcularMovimentoElastico(caso);

        return somarReacoesNormais();
    }

    /* Método chamado para subdividir a matriz das componentes das estacas */
    private void subdividirMatrizCompontesEstacas(char caso) {

        double[][] matriz1 = new double[3][MainActivity.estaqueamento.length];
        double[][] matriz2 = new double[3][MainActivity.estaqueamento.length];
        double[][] componentes = getMatrizComponentesEstacas().getArray();

        switch (caso) {

            case 'D':

                for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

                    // Matriz de componentes de estacas XYC
                    matriz1[0][i] = componentes[0][i];
                    matriz1[1][i] = componentes[1][i];
                    matriz1[2][i] = componentes[5][i];

                    // Matriz de componentes de estacas ZAB
                    matriz2[0][i] = componentes[2][i];
                    matriz2[1][i] = componentes[3][i];
                    matriz2[2][i] = componentes[4][i];
                }

                break;

            case 'E':

                for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

                    // Matriz de componentes de estacas XZB
                    matriz1[0][i] = componentes[0][i];
                    matriz1[1][i] = componentes[2][i];
                    matriz1[2][i] = componentes[4][i];

                    // Matriz de componentes de estacas YAC
                    matriz1[0][i] = componentes[1][i];
                    matriz1[1][i] = componentes[3][i];
                    matriz1[2][i] = componentes[5][i];
                }

                break;
        }

        // [p]xyc ou [p]xzb
        matrizComponentesEstacasSubdividida1 = new Matrix(matriz1);

        // [p]zab ou [p]yac
        matrizComponentesEstacasSubdividida2 = new Matrix(matriz2);
    }

    /* Método chamado para subdividir a matriz de rigidez do estaqueamento */
    private void subdividirMatrizRigidez(char caso) {

        double[][] matriz1 = new double[3][3];
        double[][] matriz2 = new double[3][3];

        switch (caso) {

            case 'D':

                // Matriz de rigidez XYC
                matriz1[0][0] = getMatrizRigidez().getArray()[0][0];
                matriz1[0][1] = getMatrizRigidez().getArray()[0][1];
                matriz1[0][2] = getMatrizRigidez().getArray()[0][5];

                matriz1[1][0] = getMatrizRigidez().getArray()[1][0];
                matriz1[1][1] = getMatrizRigidez().getArray()[1][1];
                matriz1[1][2] = getMatrizRigidez().getArray()[1][5];

                matriz1[2][0] = getMatrizRigidez().getArray()[5][0];
                matriz1[2][1] = getMatrizRigidez().getArray()[5][1];
                matriz1[2][2] = getMatrizRigidez().getArray()[5][5];

                // Matriz de rigidez ZAB
                matriz2[0][0] = getMatrizRigidez().getArray()[2][2];
                matriz2[0][1] = getMatrizRigidez().getArray()[2][3];
                matriz2[0][2] = getMatrizRigidez().getArray()[2][4];

                matriz2[1][0] = getMatrizRigidez().getArray()[3][2];
                matriz2[1][1] = getMatrizRigidez().getArray()[3][3];
                matriz2[1][2] = getMatrizRigidez().getArray()[3][4];

                matriz1[2][0] = getMatrizRigidez().getArray()[4][2];
                matriz1[2][1] = getMatrizRigidez().getArray()[4][3];
                matriz1[2][2] = getMatrizRigidez().getArray()[4][4];

                break;

            case 'E':

                // Matriz de rigidez XZB
                matriz1[0][0] = getMatrizRigidez().getArray()[0][0];
                matriz1[0][1] = getMatrizRigidez().getArray()[0][2];
                matriz1[0][2] = getMatrizRigidez().getArray()[0][4];

                matriz1[1][0] = getMatrizRigidez().getArray()[2][0];
                matriz1[1][1] = getMatrizRigidez().getArray()[2][2];
                matriz1[1][2] = getMatrizRigidez().getArray()[2][4];

                matriz1[2][0] = getMatrizRigidez().getArray()[4][0];
                matriz1[2][1] = getMatrizRigidez().getArray()[4][2];
                matriz1[2][2] = getMatrizRigidez().getArray()[4][4];

                // Matriz de rigidez YAC
                matriz1[0][0] = getMatrizRigidez().getArray()[1][1];
                matriz1[0][1] = getMatrizRigidez().getArray()[1][3];
                matriz1[0][2] = getMatrizRigidez().getArray()[1][5];

                matriz1[1][0] = getMatrizRigidez().getArray()[3][1];
                matriz1[1][1] = getMatrizRigidez().getArray()[3][3];
                matriz1[1][2] = getMatrizRigidez().getArray()[3][5];

                matriz1[2][0] = getMatrizRigidez().getArray()[5][1];
                matriz1[2][1] = getMatrizRigidez().getArray()[5][3];
                matriz1[2][2] = getMatrizRigidez().getArray()[5][5];

                break;
        }

        // [S]xyc ou [S]xzb
        matrizRigidezSubdividida1 = new Matrix(matriz1);

        // [S]zab ou [S]yac
        matrizRigidezSubdividida2 = new Matrix(matriz2);
    }

    /* Método chamado para reduzir a matriz de esforços externos */
    private void subdividirMatrizEsforcos(char caso) {

        double[][] matriz1 = new double[3][1];
        double[][] matriz2 = new double[3][1];

        switch (caso) {

            case 'D':

                // Matriz dos esforços XYC
                matriz1[0][0] = MainActivity.esforcos[0][0];
                matriz1[1][0] = MainActivity.esforcos[1][0];
                matriz1[2][0] = MainActivity.esforcos[5][0];

                // Matriz dos esforços ZAB
                matriz2[0][0] = MainActivity.esforcos[2][0];
                matriz2[1][0] = MainActivity.esforcos[3][0];
                matriz2[2][0] = MainActivity.esforcos[4][0];

                break;

            case 'E':

                // Matriz dos esforços XZB
                matriz1[0][0] = MainActivity.esforcos[0][0];
                matriz1[1][0] = MainActivity.esforcos[2][0];
                matriz1[2][0] = MainActivity.esforcos[4][0];

                // Matriz dos esforços YAC
                matriz2[0][0] = MainActivity.esforcos[1][0];
                matriz2[1][0] = MainActivity.esforcos[3][0];
                matriz2[2][0] = MainActivity.esforcos[5][0];

                break;
        }

        // [F]xyc ou [F]xzb
        matrizEsforcosExternosSubdividida1 = new Matrix(matriz1);

        // [F]zab ou [F]yac
        matrizEsforcosExternosSubdividida2 = new Matrix(matriz2);
    }

    /* Método para achar as coordenadas xo e yo dos novos eixos coordenados x' e y' após a translação
    dos eixos originais x e y, e o ângulo de rotação fi após a rotação em torno do eixo original z,
    assim como as outras novas coordenadas xoL e yoL após a translação dos eixos originais x e y, e
    o ângulo de rotação psi após a rotação em torno do eixo z como aritifício para solucionar os
    casos de estaqueamento com simetria por um plano. */
    private void acharNovasCoordenadas(char caso) {

        double[][] mS1 = matrizRigidezSubdividida1.getArray();
        double[][] mS2 = matrizRigidezSubdividida2.getArray();

        if (mS1[0][1] == 0) {

            fi = 0;

        } else {

            fi = (Math.atan((2 * mS1[0][1]) / (mS1[0][0] - mS1[1][1]))) / 2;
        }

        if ((mS2[0][0] * (mS2[1][1] - mS2[2][2]) + Math.pow(mS2[0][2], 2) - Math.pow(mS2[0][1], 2)) == 0) {

            psi = 0;

        } else {

            psi = (Math.atan(2 * (mS2[0][0] * mS2[1][2] - mS2[0][1] * mS2[0][2]) /
                    (mS2[0][0] * (mS2[1][1] - mS2[2][2]) + Math.pow(mS2[0][2], 2) - Math.pow(mS2[0][1], 2)))) / 2;
        }

        switch (caso) {

            case 'D':

                // Dados auxiliares para o sistema XYC

                xo = (mS1[0][0] * mS1[1][2] - mS1[0][1] * mS1[0][2]) / (mS1[0][0] * mS1[1][1] - Math.pow(mS1[0][1], 2));
                yo = (mS1[0][1] * mS1[1][2] - mS1[1][1] * mS1[0][2]) / (mS1[0][0] * mS1[1][1] - Math.pow(mS1[0][1], 2));

                // Dados auxiliares para os sistema ZAB

                xoL = (mS2[0][1] * Math.sin(psi) - mS2[0][2] * Math.cos(psi)) / mS2[0][0];
                yoL = (mS2[0][1] * Math.cos(psi) + mS2[0][2] * Math.sin(psi)) / mS2[0][0];

                break;

            case 'E':

                // Dados auxiliares para o sistema XZB

                xo = (mS1[0][1] * mS1[0][2] - mS1[0][0] * mS1[1][2]) / (mS1[0][0] * mS1[1][1] - Math.pow(mS1[0][1], 2));
                zo = (mS1[1][1] * mS1[0][2] - mS1[0][1] * mS1[1][2]) / (mS1[0][0] * mS1[1][1] - Math.pow(mS1[0][1], 2));

                // Dados auxiliares para o sistema YAC

                xoL = ( mS2[0][2] * Math.cos(psi) - mS2[0][1] * Math.sin(psi)) / mS2[0][0];
                zoL = (-mS2[0][2] * Math.sin(psi) - mS2[0][1] * Math.cos(psi)) / mS2[0][0];

                break;
        }
    }

    /* Método para criar a matriz de trnasformação [T], ou tensor de transformação [T], referente aos
    casos de estaquemaneto com simetria pelo plano XY ou XZ */
    private void montarTensorTransformacao(char caso) {

        double[][] matriz1 = new double[3][3];
        double[][] matriz2 = new double[3][3];

        switch (caso) {

            case 'D':

                // Tensor transformação XYZ
                matriz1[0][0] = Math.cos(fi);
                matriz1[0][1] = Math.sin(fi);
                matriz1[0][2] = 0;

                matriz1[1][0] = -Math.sin(fi);
                matriz1[1][1] = Math.cos(fi);
                matriz1[1][2] = 0;

                matriz1[2][0] = yo;
                matriz1[2][1] = -xo;
                matriz1[2][2] = 1;

                // Tensor transformação ZAB
                matriz2[0][0] = 1;
                matriz2[0][1] = 0;
                matriz2[0][2] = 0;

                matriz2[1][0] = -yoL;
                matriz2[1][1] = Math.cos(psi);
                matriz2[1][2] = Math.sin(psi);

                matriz2[2][0] = xoL;
                matriz2[2][1] = -Math.sin(psi);
                matriz2[2][2] = Math.cos(psi);

                break;

            case 'E':

                // Tensor transformação XZB
                matriz1[0][0] = Math.cos(fi);
                matriz1[0][1] = Math.sin(fi);
                matriz1[0][2] = 0;

                matriz1[1][0] = -Math.sin(fi);
                matriz1[1][1] = Math.cos(fi);
                matriz1[1][2] = 0;

                matriz1[2][0] = -zo;
                matriz1[2][1] = xo;
                matriz1[2][2] = 1;

                // Tensor transformação YAC
                matriz2[0][0] = 1;
                matriz2[0][1] = 0;
                matriz2[0][2] = 0;

                matriz2[1][0] = zoL;
                matriz2[1][1] = Math.cos(psi);
                matriz2[1][2] = Math.sin(psi);

                matriz2[2][0] = -xoL;
                matriz2[2][1] = -Math.sin(psi);
                matriz2[2][2] = Math.cos(psi);

                break;
        }

        // [T]xyc ou [T]xzb
        tensorTransformacao1 = new Matrix(matriz1);

        // [T]zab ou [T]yac
        tensorTransformacao2 = new Matrix(matriz2);
    }

    /* Método para calcular as matrizes de componentes de estaca transformados [p'], aplicando o tensor de
    transformação [T], retornando duas matrizes 3 x n: [p'] = [T] * [p],
    onde n é número de estacas */
    private void calcularComponentesEstacasTransformados() {

        // [p]'xyc ou [p]'xzb
        matrizComponentesEstacasSubTransformada1 = tensorTransformacao1.times(matrizComponentesEstacasSubdividida1);

        // [p]'zab ou [p]'yac
        matrizComponentesEstacasSubTransformada2 = tensorTransformacao2.times(matrizComponentesEstacasSubdividida2);
    }

    /* Método para calcular as matrizes de rigidez transformadas [S'], aplicando o tensor de transformação [T],
    retornando duas matrizes 3 x 3: [S'] = [T] * [S] * [T]transposta */
    private void calcularRigidezTransformada() {

        // [S]'xyc ou [S]'xzb
        matrizRigidezSubTransformada1 = tensorTransformacao1.times(matrizRigidezSubdividida1);

        // [S]'zab ou [S]'yac
        matrizRigidezSubTransformada2 = tensorTransformacao2.times(matrizRigidezSubdividida2);
    }

    /* Método para calcular as matrizes de esforços transformados [F'], aplicando o tensor de transformação [T],
    retornando duas matrizes 3 x 1: [F'] = [T] * [F] */
    private void calcularEsforcosTransformados() {

        // [F]'xyc ou [F]xzb
        matrizEsforcosSubTransformada1 = tensorTransformacao1.times(matrizEsforcosExternosSubdividida1);

        // [F]'zab ou [F]yac
        matrizEsforcosSubTransformada2 = tensorTransformacao2.times(matrizEsforcosExternosSubdividida2);
    }

    /* Método para calcular as matrizes do movimento elástico transformado do bloco [v'], após a transformação
    da matriz de rigidez [S'] e da matriz do esforços externos [F'],
    retornando duas matrizes 3 x 1: [v'] = [S' ^ -1] * [F'] */
    private void calcularMovElasticoTransformado() {

        double[][] vL1 = new double[3][1];
        double[][] vL2 = new double[3][1];

        // vx'
        if (matrizRigidezSubTransformada1.getArray()[0][0] == 0) {

            vL1[0][0] = 0;

        } else {

            vL1[0][0] = matrizEsforcosSubTransformada1.getArray()[0][0] / matrizRigidezSubTransformada1.getArray()[0][0];
        }

        // vy' ou vz'
        if (matrizRigidezSubTransformada1.getArray()[1][1] == 0) {

            vL1[1][0] = 0;

        } else {

            vL1[1][0] = matrizEsforcosSubTransformada1.getArray()[1][0] / matrizRigidezSubTransformada1.getArray()[1][1];
        }

        // vc' ou vb'
        if (matrizRigidezSubTransformada1.getArray()[2][2] == 0) {

            vL1[2][0] = 0;

        } else {

            vL1[2][0] = matrizEsforcosSubTransformada1.getArray()[2][0] / matrizRigidezSubTransformada1.getArray()[2][2];
        }

        // vz' ou vy'
        if (matrizRigidezSubTransformada2.getArray()[0][0] == 0) {

            vL2[0][0] = 0;

        } else {

            vL2[0][0] = matrizEsforcosSubTransformada2.getArray()[0][0] / matrizRigidezSubTransformada2.getArray()[0][0];
        }

        // va'
        if (matrizRigidezSubTransformada2.getArray()[1][1] == 0) {

            vL2[1][0] = 0;

        } else {

            vL2[1][0] = matrizEsforcosSubTransformada2.getArray()[1][0] / matrizRigidezSubTransformada2.getArray()[1][1];

        }

        // vb' ou vc'
        if (matrizRigidezSubTransformada2.getArray()[2][2] == 0) {

            vL2[2][0] = 0;

        } else {

            vL2[2][0] = matrizEsforcosSubTransformada2.getArray()[2][0] / matrizRigidezSubTransformada2.getArray()[2][2];
        }

        // [v]'xyc ou [v]'xzb
        matrizMovElasticoSubTransformado1 = new Matrix(vL1);

        // [v]'zab ou [v]'yac
        matrizMovElasticoSubTransformado2 = new Matrix(vL2);
    }

    private void calcularMovimentoElastico(char caso) {

        double[] matriz = new double[6];

        switch (caso) {

            case 'D':

                matriz[0] = (tensorTransformacao1.transpose()).times(matrizMovElasticoSubTransformado1).getArray()[0][0];
                matriz[1] = (tensorTransformacao1.transpose()).times(matrizMovElasticoSubTransformado1).getArray()[1][0];
                matriz[2] = (tensorTransformacao2.transpose()).times(matrizMovElasticoSubTransformado2).getArray()[0][0];
                matriz[3] = (tensorTransformacao2.transpose()).times(matrizMovElasticoSubTransformado2).getArray()[1][0];
                matriz[4] = (tensorTransformacao2.transpose()).times(matrizMovElasticoSubTransformado2).getArray()[2][0];
                matriz[5] = (tensorTransformacao1.transpose()).times(matrizMovElasticoSubTransformado1).getArray()[2][0];

                break;

            case 'E':

                matriz[0] = (tensorTransformacao1.transpose()).times(matrizMovElasticoSubTransformado1).getArray()[0][0];
                matriz[1] = (tensorTransformacao2.transpose()).times(matrizMovElasticoSubTransformado2).getArray()[0][0];
                matriz[2] = (tensorTransformacao1.transpose()).times(matrizMovElasticoSubTransformado1).getArray()[1][0];
                matriz[3] = (tensorTransformacao2.transpose()).times(matrizMovElasticoSubTransformado2).getArray()[1][0];
                matriz[4] = (tensorTransformacao1.transpose()).times(matrizMovElasticoSubTransformado1).getArray()[2][0];
                matriz[5] = (tensorTransformacao2.transpose()).times(matrizMovElasticoSubTransformado2).getArray()[2][0];

                break;
        }

        MainActivity.movElastico = matriz;
    }

    /* Método para somar os valores das reações encontradas pelas subdivisões do cálculo de estaqueamento  */
    private Matrix somarReacoesNormais() {

        double[][] matriz1 = ((getMatrizRigidezEstacas().times(matrizComponentesEstacasSubTransformada1.transpose())).times(matrizMovElasticoSubTransformado1)).getArray();
        double[][] matriz2 = ((getMatrizRigidezEstacas().times(matrizComponentesEstacasSubTransformada2.transpose())).times(matrizMovElasticoSubTransformado2)).getArray();

        double[][] matriz = new double[MainActivity.estaqueamento.length][1];

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            matriz[i][0] = matriz1[i][0] + matriz2[i][0];
        }

        return new Matrix(matriz);
    }
}