import Jama.Matrix;

public class SimetriaPorUmEixo extends AnalisesPreliminares {

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
    private Matrix matrizMovElastico1;

    private Matrix matrizComponentesEstacasSubdividida3;
    private Matrix matrizComponentesEstacasSubdividida4;
    private Matrix matrizRigidezSubdividida3;
    private Matrix matrizRigidezSubdividida4;
    private Matrix matrizEsforcosExternosSubdividida3;
    private Matrix matrizEsforcosExternosSubdividida4;
    private Matrix matrizMovElasticoSubTransformado3;
    private Matrix matrizMovElasticoSubTransformado4;
    private Matrix matrizMovElasticoTransformado;

    private double fi;
    private double psi;

    private double xy;
    private double xz;

    /* Método para calcular os esforços normais de reação nas estacas: [N] = [v'']transposta * [p''] */
    public Matrix calcularEsforcosNormais() {

        subdividirMatrizCompontesEstacas();
        subdividirMatrizRigidez();
        subdividirMatrizEsforcos();

        acharNovasCoordenadas1();
        tensorTransformacao1 = montarTensorTransformacao1();
        matrizComponentesEstacasSubTransformada1 = calcularComponentesSubTransformados1();
        matrizRigidezSubTransformada1 = calcularRigidezTransformada1();
        matrizEsforcosSubTransformada1 = calcularEsforcosSubTransformados1();

        acharNovasCoordenadas2();
        tensorTransformacao2 = montarTensorTransformacao2();
        matrizComponentesEstacasSubTransformada2 = calcularComponentesSubTransformados2();
        matrizRigidezSubTransformada2 = calcularRigidezTransformada2();
        matrizEsforcosSubTransformada2 = calcularEsforcosSubTransformados2();

        subdividirMatrizCompontesEstacas2();
        subdividirMatrizRigidez2();
        subdividirMatrizEsforcos2();

        calcularMovElasticoTransformado();

        calcularMovimentoElastico();

        return somarReacoesNormais();
    }

    /* Método chamado para subdividir a matriz das componentes das estacas */
    private void subdividirMatrizCompontesEstacas() {

        double[][] matriz1 = new double[2][MainActivity.estaqueamento.length];
        double[][] matriz2 = new double[4][MainActivity.estaqueamento.length];
        double[][] componentes = getMatrizComponentesEstacas().getArray();

            for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

                // Matriz de componentes de estacas XA
                matriz1[0][i] = componentes[0][i];
                matriz1[1][i] = componentes[3][i];

                // Matriz de componentes de estacas YZBC
                matriz2[0][i] = componentes[1][i];
                matriz2[1][i] = componentes[2][i];
                matriz2[2][i] = componentes[4][i];
                matriz2[3][i] = componentes[5][i];
            }

        // [p]xa
        matrizComponentesEstacasSubdividida1 = new Matrix(matriz1);

        // [p]yzbc
        matrizComponentesEstacasSubdividida2 = new Matrix(matriz2);
    }

    /* Método chamado para subdividir a matriz de rigidez do estaqueamento */
    private void subdividirMatrizRigidez() {

        double[][] matriz1 = new double[2][2];
        double[][] matriz2 = new double[4][4];

        // Matriz de rigidez XA
        matriz1[0][0] = getMatrizRigidez().getArray()[0][0];
        matriz1[0][1] = getMatrizRigidez().getArray()[0][3];

        matriz1[1][0] = getMatrizRigidez().getArray()[3][0];
        matriz1[1][1] = getMatrizRigidez().getArray()[3][3];

        // Matriz de rigidez YZBC
        matriz2[0][0] = getMatrizRigidez().getArray()[1][1];
        matriz2[0][1] = getMatrizRigidez().getArray()[1][2];
        matriz2[0][2] = getMatrizRigidez().getArray()[1][4];
        matriz2[0][3] = getMatrizRigidez().getArray()[1][5];

        matriz2[1][0] = getMatrizRigidez().getArray()[2][1];
        matriz2[1][1] = getMatrizRigidez().getArray()[2][2];
        matriz2[1][2] = getMatrizRigidez().getArray()[2][4];
        matriz2[1][3] = getMatrizRigidez().getArray()[2][5];

        matriz2[2][0] = getMatrizRigidez().getArray()[4][1];
        matriz2[2][1] = getMatrizRigidez().getArray()[4][2];
        matriz2[2][2] = getMatrizRigidez().getArray()[4][4];
        matriz2[2][3] = getMatrizRigidez().getArray()[4][5];

        matriz2[3][0] = getMatrizRigidez().getArray()[5][1];
        matriz2[3][1] = getMatrizRigidez().getArray()[5][2];
        matriz2[3][2] = getMatrizRigidez().getArray()[5][4];
        matriz2[3][3] = getMatrizRigidez().getArray()[5][5];

        // [S]xa
        matrizRigidezSubdividida1 = new Matrix(matriz1);

        // [S]yzbc
        matrizRigidezSubdividida2 = new Matrix(matriz2);
    }

    /* Método chamado para reduzir a matriz de esforços externos */
    private void subdividirMatrizEsforcos() {

        double[][] matriz1 = new double[2][1];
        double[][] matriz2 = new double[4][1];

        // Matriz dos esforços XA
        matriz1[0][0] = MainActivity.esforcos[0][0];
        matriz1[1][0] = MainActivity.esforcos[3][0];

        // Matriz dos esforços YZBC
        matriz2[0][0] = MainActivity.esforcos[1][0];
        matriz2[1][0] = MainActivity.esforcos[2][0];
        matriz2[2][0] = MainActivity.esforcos[4][0];
        matriz2[3][0] = MainActivity.esforcos[5][0];

        // [F]xa
        matrizEsforcosExternosSubdividida1 = new Matrix(matriz1);

        // [F]yzbc
        matrizEsforcosExternosSubdividida2 = new Matrix(matriz2);
    }

    /* Método para achar o ângulo de rotação fi, após a rotação dos antigos eixos coordenados
    como aritifício para solucionar casos de estaqueamento simétrico por um plano. */
    private void acharNovasCoordenadas1() {

        double[][] mS = matrizRigidezSubdividida2.getArray();

        if (mS[0][1] == 0) {

            fi = 0;

        } else if ((mS[0][0] - mS[1][1]) == 0) {

            fi = 0;

        } else {

            fi = (Math.atan((2 * mS[0][1]) / (mS[0][0] - mS[1][1]))) / 2;
        }
    }

    /* Método para criar a primeira matriz de trnasformação [T], ou tensor de transformação [T],
    referente aos casos de estaquemaneto com simetria por um eixo */
    private Matrix montarTensorTransformacao1() {

        double[][] tensorT = new double[4][4];

        tensorT[0][0] = Math.cos(fi);
        tensorT[0][1] = Math.sin(fi);
        tensorT[0][2] = 0;
        tensorT[0][3] = 0;

        tensorT[1][0] = -Math.sin(fi);
        tensorT[1][1] = Math.cos(fi);
        tensorT[1][2] = 0;
        tensorT[1][3] = 0;

        tensorT[2][0] = 0;
        tensorT[2][1] = 0;
        tensorT[2][2] = Math.cos(fi);
        tensorT[2][3] = Math.sin(fi);

        tensorT[3][0] = 0;
        tensorT[3][1] = 0;
        tensorT[3][2] = -Math.sin(fi);
        tensorT[3][3] = Math.cos(fi);

        return new Matrix(tensorT);
    }

    /* Método para calcular a matriz componentes de estaca transformados [p'], aplicando o tensor de
    transformação [T], retornando duas matrizes 4 x n: [p'] = [T] * [p],
    onde n é número de estacas */
    private Matrix calcularComponentesSubTransformados1() {

        return tensorTransformacao1.times(matrizComponentesEstacasSubdividida2);
    }

    /* Método para calcular a matriz de rigidez transformada [S'], aplicando o tensor de transformação [T],
    retornando uma matriz 4 x 4: [S'] = [T] * [S] * [T]transposta */
    private Matrix calcularRigidezTransformada1() {

        return tensorTransformacao1.times(matrizRigidezSubdividida2.times(tensorTransformacao1.transpose()));
    }

    /* Método para calcular a matriz de esforços transformados [F'], aplicando o tensor de transformação [T],
     retornando uma matriz 4 x 1: [F'] = [T] * [F] */
    private Matrix calcularEsforcosSubTransformados1() {

        return tensorTransformacao1.times(matrizEsforcosExternosSubdividida2);
    }

    /* Método para achar o ângulo de rotação psi, após a rotação dos eixos coordenados após a primeira
    transformação e as posições xy e xz dos novos eixos y e z, após a segunda transformação
    como aritifício para solucionar casos de estaqueamento simétrico por um plano. */
    private void acharNovasCoordenadas2() {

        double[][] mS = matrizRigidezSubTransformada1.getArray();

        if (mS[3][2] == 0) {

            psi = 0;

        } else if (Math.pow(mS[0][3],2) * mS[1][1] + Math.pow(mS[1][3],2) * mS[0][0] + mS[2][2] * mS[0][0] * mS[1][1]
                - mS[3][3] * mS[0][0] * mS[1][1] - Math.pow(mS[0][2],2) * mS[1][1] + Math.pow(mS[1][2],2) * mS[0][0] < Math.pow(10,-10)) {

            psi = 0;

        } else {

            psi = (Math.atan(mS[2][3] * mS[0][0] * mS[1][1] - mS[1][2] * mS[1][3] * mS[0][0] - mS[0][2] * mS[0][3] * mS[1][1]) /
                    (Math.pow(mS[0][3],2) * mS[1][1] + Math.pow(mS[1][3],2) * mS[0][0] + mS[2][2] * mS[0][0] * mS[1][1]
                    - mS[3][3] * mS[0][0] * mS[1][1] - Math.pow(mS[0][2],2) * mS[1][1] + Math.pow(mS[1][2],2) * mS[0][0]))/2;
        }

        xy = (mS[0][3] * Math.cos(psi) - mS[0][2] * Math.sin(psi)) / mS[0][0];

        xz = -(mS[1][2] * Math.cos(psi + mS[1][3] * Math.sin(psi))) / mS[1][1];
    }

    /* Método para criar a segunda matriz de trnasformação [T], ou tensor de transformação [T],
    referente aos casos de estaquemaneto com simetria por um eixo */
    private Matrix montarTensorTransformacao2() {

        double[][] tensorT = new double[4][4];

        tensorT[0][0] = 1;
        tensorT[0][1] = 0;
        tensorT[0][2] = 0;
        tensorT[0][3] = 0;

        tensorT[1][0] = 0;
        tensorT[1][1] = 1;
        tensorT[1][2] = 0;
        tensorT[1][3] = 0;

        tensorT[2][0] = 0;
        tensorT[2][1] = xz;
        tensorT[2][2] = Math.cos(psi);
        tensorT[2][3] = Math.sin(psi);

        tensorT[3][0] = -xy;
        tensorT[3][1] = 0;
        tensorT[3][2] = -Math.sin(psi);
        tensorT[3][3] = Math.cos(psi);

        return new Matrix(tensorT);
    }

    /* Método para calcular a matriz componentes de estaca transformados [p''], aplicando o tensor de
    transformação [T], retornando duas matrizes 4 x n: [p''] = [T] * [p'],
    onde n é número de estacas */
    private Matrix calcularComponentesSubTransformados2() {

        return tensorTransformacao2.times(matrizComponentesEstacasSubTransformada1);
    }

    /* Método para calcular a matriz de rigidez transformada [S''], aplicando o tensor de transformação [T],
    retornando uma matriz 4 x 4: [S''] = [T] * [S'] * [T]transposta */
    private Matrix calcularRigidezTransformada2() {

        return tensorTransformacao2.times(matrizRigidezSubTransformada1.times(tensorTransformacao2.transpose()));
    }

    /* Método para calcular a matriz de esforços transformados [F''], aplicando o tensor de transformação [T],
     retornando uma matriz 4 x 1: [F''] = [T] * [F'] */
    private Matrix calcularEsforcosSubTransformados2() {

        return tensorTransformacao2.times(matrizEsforcosSubTransformada1);
    }

    /* Método para subdividir a segunda matriz componente de estacas subdividida transformada */
    private void subdividirMatrizCompontesEstacas2() {

        double[][] matriz1 = new double[2][MainActivity.estaqueamento.length];
        double[][] matriz2 = new double[2][MainActivity.estaqueamento.length];

        double[][] componentes = matrizComponentesEstacasSubTransformada2.getArray();

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            // Matriz de componentes de estacas YB'
            matriz1[0][i] = componentes[0][i];
            matriz1[1][i] = componentes[2][i];

            // Matriz de componentes de estacas ZC'
            matriz2[0][i] = componentes[1][i];
            matriz2[1][i] = componentes[3][i];
        }

        // [p]yb'
        matrizComponentesEstacasSubdividida3 = new Matrix(matriz1);

        // [p]zc'
        matrizComponentesEstacasSubdividida4 = new Matrix(matriz2);
    }

    /* Método para subdividir a segunda matriz de rigidez subdividida transformada */
    private void subdividirMatrizRigidez2() {

        double[][] matriz1 = new double[2][2];
        double[][] matriz2 = new double[2][2];

        // Matriz de rigidez YB'
        matriz1[0][0] = matrizRigidezSubTransformada2.getArray()[0][0];
        matriz1[0][1] = matrizRigidezSubTransformada2.getArray()[0][2];

        matriz1[1][0] = matrizRigidezSubTransformada2.getArray()[2][0];
        matriz1[1][1] = matrizRigidezSubTransformada2.getArray()[2][2];

        // Matriz de rigidez ZC'
        matriz2[0][0] = matrizRigidezSubTransformada2.getArray()[1][1];
        matriz2[0][1] = matrizRigidezSubTransformada2.getArray()[1][3];

        matriz2[1][0] = matrizRigidezSubTransformada2.getArray()[3][1];
        matriz2[1][1] = matrizRigidezSubTransformada2.getArray()[3][3];

        // [S]xa
        matrizRigidezSubdividida3 = new Matrix(matriz1);

        // [S]yzbc
        matrizRigidezSubdividida4 = new Matrix(matriz2);
    }

    /* Método para subdividir a segunda matriz de esforços subdividida transformada */
    private void subdividirMatrizEsforcos2() {

        double[][] matriz1 = new double[2][1];
        double[][] matriz2 = new double[2][1];

        // Matriz dos esforços YB'
        matriz1[0][0] = matrizEsforcosSubTransformada2.getArray()[0][0];
        matriz1[1][0] = matrizEsforcosSubTransformada2.getArray()[2][0];

        // Matriz dos esforços ZC'
        matriz2[0][0] = matrizEsforcosSubTransformada2.getArray()[1][0];
        matriz2[1][0] = matrizEsforcosSubTransformada2.getArray()[3][0];

        // [F]yb'
        matrizEsforcosExternosSubdividida3 = new Matrix(matriz1);

        // [F]zc'
        matrizEsforcosExternosSubdividida4 = new Matrix(matriz2);
    }

    /* Método para calcular as matrizes do movimento elástico transformado do bloco [v'], após a transformação
    da matriz de rigidez [S''] e da matriz do esforços externos [F''],
    retornando três matrizes 2 x 1: [v''] = [S'' ^ -1] * [F''] */
    private void calcularMovElasticoTransformado() {

        double[][] matriz = new double[4][1];
        double[][] matriz1 = {{0}, {0}};
        double[][] matriz3 = {{0}, {0}};
        double[][] matriz4 = {{0}, {0}};

        Matrix vxa = new Matrix(matriz1);
        Matrix vybL = new Matrix(matriz3);
        Matrix vzcL = new Matrix(matriz4);

        try {

            vxa = matrizRigidezSubdividida1.solve(matrizEsforcosExternosSubdividida1);

        } catch (Exception e) {

            // mandar mensagem sobre hiperestaticidade
        }

        try {

            vybL = matrizRigidezSubdividida3.solve(matrizEsforcosExternosSubdividida3);


        } catch (Exception e) {

            // mandar mensagem sobre hiperestaticidade
        }

        try {

            vzcL = matrizRigidezSubdividida4.solve(matrizEsforcosExternosSubdividida4);

        } catch (Exception e) {

            // mandar mensagem sobre hiperestaticidade
        }

        matriz[0][0] = vybL.getArray()[0][0];
        matriz[1][0] = vzcL.getArray()[0][0];
        matriz[2][0] = vybL.getArray()[1][0];
        matriz[3][0] = vzcL.getArray()[1][0];

        matrizMovElasticoTransformado = new Matrix(matriz);

        // [v]xa
        matrizMovElastico1 = vxa;

        // [v]''yb
        matrizMovElasticoSubTransformado3 = vybL;

        // [v]''zc
        matrizMovElasticoSubTransformado4 = vzcL;
    }

    private void calcularMovimentoElastico() {

        double[] matriz = new double[6];

        Matrix movElasticoTransformado = (tensorTransformacao1.transpose()).times((tensorTransformacao2.transpose()).times(matrizMovElasticoTransformado));

        matriz[0] = matrizMovElastico1.getArray()[0][0];
        matriz[1] = movElasticoTransformado.getArray()[0][0];
        matriz[2] = movElasticoTransformado.getArray()[1][0];
        matriz[3] = matrizMovElastico1.getArray()[1][0];
        matriz[4] = movElasticoTransformado.getArray()[2][0];
        matriz[5] = movElasticoTransformado.getArray()[3][0];

        MainActivity.movElastico = matriz;
    }

    /* Método para somar os valores das reações encontradas pelas subdivisões do cálculo de estaqueamento  */
    private Matrix somarReacoesNormais() {

        double[][] matriz1 = ((getMatrizRigidezEstacas().times(matrizComponentesEstacasSubdividida1.transpose())).times(matrizMovElastico1)).getArray();
        double[][] matriz2 = ((getMatrizRigidezEstacas().times(matrizComponentesEstacasSubdividida3.transpose())).times(matrizMovElasticoSubTransformado3)).getArray();
        double[][] matriz3 = ((getMatrizRigidezEstacas().times(matrizComponentesEstacasSubdividida4.transpose())).times(matrizMovElasticoSubTransformado4)).getArray();
        double[][] matriz = new double[MainActivity.estaqueamento.length][1];

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            matriz[i][0] = matriz1[i][0] + matriz2[i][0] + matriz3[i][0];
        }

        return new Matrix(matriz);
    }
}